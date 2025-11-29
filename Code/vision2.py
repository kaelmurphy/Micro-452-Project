import cv2
import threading
import numpy as np
import math
import time

# CONSTANTS
# ===========================================================
ROBOT_ID = 8
GOAL_ID = 9

# GLOBAL VISION STATE
# ===========================================================
VISION_CAMERA = None  # cached CameraStream
VISION_H = None       # cached pixel->world homography


# CAMERA STREAM
# ============================================================

class CameraStream:
    def __init__(self, width=640, height=480, fps=30):
        self.index = 0  # Hardcoded camera index
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {self.index}.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.frame = None

    def start(self):
        if self.running:
            return self
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1):
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                with self.lock:
                    self.frame = frame
            time.sleep(0.01)

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.cap.release()


# ARUCO DETECTION & POSE
# ============================================================

def detectAruco(frame, dictName="DICT_4X4_50"):
    if not hasattr(cv2, "aruco"):
        return [], {}, {}, frame

    dictAttr = getattr(cv2.aruco, dictName, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dictAttr)
    detector = cv2.aruco.ArucoDetector(dictionary)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    centerMap = {}
    cornerMap = {}
    idList = []

    if ids is not None:
        idList = ids.flatten().tolist()
        for i, markerId in enumerate(idList):
            pts = corners[i][0]  # (4,2)
            cornerMap[markerId] = pts.astype(int)
            center = pts.mean(axis=0).astype(int)
            centerMap[markerId] = tuple(center)

    return idList, centerMap, cornerMap, frame  # Skip annotated frame for speed


def buildZone(centerMap):
    required = [0, 1, 2, 3]
    missing = [i for i in required if i not in centerMap]
    # corners in order [bl, br, tr, tl]
    cornerOrder = [3, 2, 1, 0]
    return {
        "isComplete": len(missing) == 0,
        "missing": missing,
        "corners": [centerMap[i] for i in cornerOrder if i in centerMap],
    }


# DRAWING UTILITIES
# ============================================================

def drawZone(frame, zone, color=(0, 255, 255)):
    if not zone.get("corners"):
        return frame
    pts = np.array(zone["corners"], dtype=np.int32)
    cv2.polylines(frame, [pts], True, color, 1)
    return frame


def drawRobot(canvas, center, corners=None):
    """Draw robot marker with orientation arrow (pixel space only)."""
    robotCenter = (int(center[0]), int(center[1]))
    cv2.circle(canvas, robotCenter, 6, (255, 0, 0), -1)

    if corners is not None:
        topMid = ((corners[0] + corners[1]) / 2).astype(int)
        direction = topMid - robotCenter
        arrowEnd = robotCenter + (direction * 0.9).astype(int)
        cv2.arrowedLine(canvas, robotCenter, tuple(arrowEnd), (255, 150, 150), 2)


def drawGoal(canvas, center):
    goalCenter = (int(center[0]), int(center[1]))
    cv2.circle(canvas, goalCenter, 8, (0, 255, 0), -1)


def drawObstacles(canvas, obstacles):
    if not obstacles:
        return canvas

    for obs in obstacles:
        cnt = np.array(obs.contour, dtype=np.int32)
        cv2.fillPoly(canvas, [cnt], (0, 0, 255))
        cv2.polylines(canvas, [cnt], True, (0, 0, 255), 1)

    return canvas


# HOMOGRAPHY HELPERS (A0 SHEET)
# ============================================================

def computeHomography(zone):
    """
    Compute pixel->world homography H using the zone corners.
    Markers 3,2,1,0 are at A0 corners in landscape orientation.

    World (mm):
        (0,   0)   : ID 3  bottom-left
        (1189,0)   : ID 2  bottom-right
        (1189,841) : ID 1  top-right
        (0,  841)  : ID 0  top-left
    """
    if (not zone.get("isComplete")
            or "corners" not in zone
            or len(zone["corners"]) != 4):
        return None

    src = np.array(zone["corners"], dtype=np.float32)

    widthMm = 1255.0
    heightMm = 740

    dst = np.array(
        [
            [0.0, 0.0],
            [widthMm, 0.0],
            [widthMm, heightMm],
            [0.0, heightMm],
        ],
        dtype=np.float32,
    )

    H = cv2.getPerspectiveTransform(src, dst)
    return H


def pixelToWorld(pt, H):
    """Map a single pixel point (x,y) to world mm using homography H."""
    if H is None or pt is None:
        return None
    x, y = float(pt[0]), float(pt[1])
    src = np.array([[[x, y]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, H)[0, 0]
    return float(dst[0]), float(dst[1])


# OBSTACLE CLASS & PROCESSING
# ============================================================

class Obstacle:
    def __init__(self, contour, area):
        self.contour = contour
        self.area = area
        self._verts = None

    @property
    def verts(self):
        """
        More stable polygon extraction:
        - epsilon for approxPolyDP
        - snap to small pixel grid
        - merge very close vertices
        """
        if self._verts is None:
            peri = cv2.arcLength(self.contour, True)
            epsilon = 0.02 * peri
            approx = cv2.approxPolyDP(self.contour, epsilon, True)

            pts = [(float(p[0][0]), float(p[0][1])) for p in approx]

            grid = 3
            snapped = [
                (int(round(x / grid) * grid), int(round(y / grid) * grid))
                for (x, y) in pts
            ]

            merged = []
            mergeDist = 3
            for x, y in snapped:
                if not merged:
                    merged.append([x, y])
                    continue

                found = False
                for v in merged:
                    dx = x - v[0]
                    dy = y - v[1]
                    if dx * dx + dy * dy <= mergeDist * mergeDist:
                        v[0] = int(round((v[0] + x) / 2))
                        v[1] = int(round((v[1] + y) / 2))
                        found = True
                        break

                if not found:
                    merged.append([x, y])

            self._verts = [(vx, vy) for vx, vy in merged]

        return self._verts

    def getWorldVerts(self, H=None):
        """Return obstacle vertices in world (A0 mm) coordinates using homography H."""
        if H is None:
            return []
        worldVerts = [pixelToWorld(v, H) for v in self.verts]
        return self._sortVertsCcwFromBl(worldVerts)

    def _sortVertsCcwFromBl(self, verts):
        if len(verts) < 3:
            return verts

        cx = sum(v[0] for v in verts) / len(verts)
        cy = sum(v[1] for v in verts) / len(verts)

        def angle(v):
            ang = math.atan2(v[1] - cy, v[0] - cx)
            return ang + 2 * math.pi if ang < 0 else ang

        sortedVerts = sorted(verts, key=angle)
        blIdx = min(
            range(len(sortedVerts)),
            key=lambda i: (sortedVerts[i][1], sortedVerts[i][0]),
        )
        return sortedVerts[blIdx:] + sortedVerts[:blIdx]

    def toDict(self):
        return {"contour": self.contour.tolist(), "area": self.area, "vertices": self.verts}


def detectObstacles(frame, zone, minArea=400, maxArea=50000):
    """
    Detect colored obstacles with optimized processing pipeline.
    """
    if not zone.get("isComplete"):
        return []

    height, width = frame.shape[:2]
    zoneMask = np.zeros((height, width), dtype=np.uint8)
    zoneCorners = zone.get("corners", [])
    if len(zoneCorners) >= 3:
        zonePts = np.array(zoneCorners, dtype=np.int32)
        cv2.fillPoly(zoneMask, [zonePts], 255)
    else:
        zoneMask[:] = 255

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    lowerObs = np.array([60, 60, 30])
    upperObs = np.array([150, 150, 100])
    mask = cv2.inRange(blurred, lowerObs, upperObs)

    mask = cv2.bitwise_and(mask, zoneMask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if minArea <= area <= maxArea and len(cnt) >= 3:
            obstacles.append(Obstacle(cnt, area))
    return obstacles


# COORDINATE CAPTURE
# ============================================================

def getVisionCoords(timeout=None, showDisplay=True):
    """
    Get coordinate data from the vision system.

    Returns:
        (coords, robotThetaWorld)
        coords: numpy array [type, id, label, x, y] in A0 mm
        robotThetaWorld: float angle in radians in A0 world frame, or None
        H transformation homography matrix

    Orientation convention:
        - 0 rad = along +X of homographic/world axis (to the right)
        - angle increases counter-clockwise
        - theta is wrapped into [0, 2π)
    """
    global VISION_CAMERA, VISION_H

    bufferLen = 15
    coordBuf = []
    camera = None

    try:
        camera = CameraStream(width=640, height=480, fps=30)
        camera.start()
        time.sleep(1)

        if showDisplay:
            cv2.namedWindow("Vision System", cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("Obstacle Debug Frame", cv2.WINDOW_AUTOSIZE)

        startTime = time.time()
        while True:
            robotThetaWorld = None

            if timeout is not None:
                elapsed = time.time() - startTime
                if elapsed > timeout:
                    if camera is not None:
                        camera.stop()
                    if showDisplay:
                        cv2.destroyAllWindows()
                    return np.array([]).reshape(0, 5), None

            frame = camera.read() if camera is not None else None
            if frame is None:
                time.sleep(2)
                continue

            dispFrame = frame.copy()
            obsFrame = frame.copy()

            try:
                idList, centerMap, cornerMap, _ = detectAruco(frame)
                zone = buildZone(centerMap)

                H = computeHomography(zone)

                # White paintover for robot + shadow in obstacle frame
                if ROBOT_ID in centerMap and ROBOT_ID in cornerMap:
                    cx, cy = centerMap[ROBOT_ID]
                    robotCorners = cornerMap[ROBOT_ID].astype(np.int32)
                    x, y, w, h = cv2.boundingRect(robotCorners)
                    radius = int(0.7 * max(w, h))

                    cv2.circle(
                        obsFrame,
                        (int(cx), int(cy)),
                        radius,
                        (255, 255, 255),
                        thickness=-1,
                    )

                if GOAL_ID in centerMap and GOAL_ID in cornerMap:
                    gx, gy = centerMap[GOAL_ID]
                    goalCorners = cornerMap[GOAL_ID].astype(np.int32)
                    xg, yg, wg, hg = cv2.boundingRect(goalCorners)
                    radius_g = int(0.7 * max(wg, hg))

                    cv2.circle(
                        obsFrame,
                        (int(gx), int(gy)),
                        radius_g,
                        (255, 255, 255),
                        thickness=-1,
                    )

                # Robot orientation in world frame (0 = right, CCW, wrapped to [0, 2π))
                if (ROBOT_ID in centerMap) and (H is not None) and (ROBOT_ID in cornerMap):
                    robotWorld = pixelToWorld(centerMap[ROBOT_ID], H)

                    rCorners = cornerMap[ROBOT_ID].astype(float)
                    topMidPx = (rCorners[0] + rCorners[1]) / 2.0
                    topMidWorld = pixelToWorld(topMidPx, H)

                    dx = topMidWorld[0] - robotWorld[0]
                    dy = topMidWorld[1] - robotWorld[1]
                    robotThetaWorld = np.atan2(dy, dx)

                obstacles = []
                if zone.get("isComplete"):
                    obstacles = detectObstacles(obsFrame, zone, minArea=400)

                if showDisplay:
                    if zone.get("isComplete") and zone.get("corners"):
                        dispFrame = drawZone(dispFrame, zone, color=(0, 255, 255))
                    if ROBOT_ID in centerMap:
                        drawRobot(dispFrame, centerMap[ROBOT_ID], cornerMap.get(ROBOT_ID))
                    if GOAL_ID in centerMap:
                        drawGoal(dispFrame, centerMap[GOAL_ID])
                    if obstacles:
                        dispFrame = drawObstacles(dispFrame, obstacles)

                    cv2.imshow("Vision System", dispFrame)
                    cv2.imshow("Obstacle Debug Frame", obsFrame)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        if camera is not None:
                            camera.stop()
                        cv2.destroyAllWindows()
                        return np.array([]).reshape(0, 5), None, None

                coords = _extractFrameCoords(frame, centerMap, cornerMap, zone, obstacles, H)

                types = coords[:, 0] if coords.size > 0 else []
                hasCorners = np.sum(types == "square") >= 4
                hasRobot = np.any(types == "start")
                hasGoal = np.any(types == "goal")
                hasObstacle = np.any(types == "vertex")

                def obstaclesAreSimilar(obsArr1, obsArr2, tol=50.0):
                    if len(obsArr1) != len(obsArr2):
                        return False
                    for r1, r2 in zip(obsArr1, obsArr2):
                        if r1[2] != r2[2] or r1[1] != r2[1]:
                            return False
                        x1, y1 = round(r1[3], 1), round(r1[4], 1)
                        x2, y2 = round(r2[3], 1), round(r2[4], 1)
                        if abs(x1 - x2) > tol or abs(y1 - y2) > tol:
                            return False
                    return True

                if coords.size > 0 and hasCorners and hasRobot and hasGoal and hasObstacle:
                    obstacleRows = [row for row in coords if row[0] == "vertex"]
                    if len(coordBuf) == 0 or obstaclesAreSimilar(
                        obstacleRows,
                        [row for row in coordBuf[-1] if row[0] == "vertex"],
                    ):
                        coordBuf.append(coords.copy())
                        if len(coordBuf) > bufferLen:
                            coordBuf = coordBuf[-bufferLen:]

                        if (
                            len(coordBuf) == bufferLen
                            and all(
                                obstaclesAreSimilar(
                                    [row for row in c if row[0] == "vertex"],
                                    [row for row in coordBuf[0] if row[0] == "vertex"],
                                )
                                for c in coordBuf
                            )
                        ) or (
                            len(coordBuf) == bufferLen
                            and hasCorners
                            and hasRobot
                            and hasGoal
                            and hasObstacle
                        ):
                            # Cache camera and homography for fast tracking.
                            VISION_CAMERA = camera   # keep it running
                            VISION_H = H

                            if showDisplay:
                                cv2.destroyAllWindows()

                            return coordBuf[0], robotThetaWorld, H
                    else:
                        coordBuf = [coords.copy()]
            except Exception as e:
                print(f"[DEBUG] Exception in main loop: {e}")
    except Exception:
        try:
            if camera is not None:
                camera.stop()
            if showDisplay:
                cv2.destroyAllWindows()
        except Exception:
            pass
        return np.array([]).reshape(0, 5), None, None


def _extractFrameCoords(frame, centerMap, cornerMap, zone, obstacles=None, H=None):
    """Extract all coordinates from a stable frame in world mm coordinates."""
    coordList = []

    if H is None:
        return np.array([]).reshape(0, 5)

    try:
        if zone.get("isComplete"):
            cornersPx = zone["corners"]
            cornerLabels = ["poly0"] * 4
            cornerNames = ["3", "2", "1", "0"]
            cornersWorld = [pixelToWorld(pt, H) for pt in cornersPx]

            if (
                isinstance(cornersWorld, (list, tuple, np.ndarray))
                and len(cornersWorld) > 0
                and hasattr(cornersWorld[0], "__iter__")
            ):   
                for (x, y), polyId, label in zip(cornersWorld, cornerLabels, cornerNames):
                    coordList.append(["square", polyId, label, float(x), float(y)])

        if ROBOT_ID in centerMap:
            robotWorld = pixelToWorld(centerMap[ROBOT_ID], H)
            if isinstance(robotWorld, tuple) and len(robotWorld) == 2:
                coordList.append(
                    [
                        "start",
                        "start_pt",
                        "Start",
                        float(robotWorld[0]),
                        float(robotWorld[1]),
                    ]
                )

        if GOAL_ID in centerMap:
            goalWorld = pixelToWorld(centerMap[GOAL_ID], H)
            if isinstance(goalWorld, tuple) and len(goalWorld) == 2:
                coordList.append(
                    [
                        "goal",
                        "goal_pt",
                        "Goal",
                        float(goalWorld[0]),
                        float(goalWorld[1]),
                    ]
                )

        if obstacles is None:
            obstacles = detectObstacles(frame, zone, minArea=400)
        if obstacles is None:
            obstacles = []

        for idx, obs in enumerate(obstacles):
            worldVerts = obs.getWorldVerts(H)
            if isinstance(worldVerts, tuple) and len(worldVerts) == 2:
                worldVerts = [worldVerts]
            if isinstance(worldVerts, np.ndarray):
                worldVerts = worldVerts.tolist()
            polyId = f"poly{idx + 1}"
            for vIdx, (x, y) in enumerate(worldVerts):
                coordList.append(
                    [
                        "vertex",
                        polyId,
                        str(vIdx + 1),
                        float(x),
                        float(y),
                    ]
                )

        return np.array(coordList, dtype=object) if coordList else np.array([]).reshape(0, 5)

    except Exception:
        print("No coordinates extracted from frame due to error.")
        return np.array([]).reshape(0, 5)


# FAST ROBOT POSE QUERY
# ============================================================

def getRobotPositionMm(showDisplay=False):
    """
    Lightweight function to get the robot's current pose in world mm.

    This will BLOCK until the robot is detected; if the robot is
    not visible, it keeps trying forever.

    Returns:
        (x_mm, y_mm, theta_world_rad)

        - x_mm, y_mm in homographic world mm
        - theta_world_rad in [0, 2π), with:
            0    = facing +X (to the right along bottom horizontal)
            π/2  = facing +Y (up)
            π    = facing -X (left)
            3π/2 = facing -Y (down)
    """
    global VISION_CAMERA, VISION_H

    if VISION_CAMERA is None or VISION_H is None:
        raise RuntimeError(
            "Vision not initialized. Call getVisionCoords(...) once first "
            "to lock in the zone and homography."
        )

    while True:
        frame = VISION_CAMERA.read()
        if frame is None:
            time.sleep(0.01)
            continue

        _, centerMap, cornerMap, _ = detectAruco(frame)

        if ROBOT_ID not in centerMap:
            time.sleep(0.01)
            return None, None, None, False

        # Robot position in world mm using cached homography
        robotWorld = pixelToWorld(centerMap[ROBOT_ID], VISION_H)
        if robotWorld is None:
            time.sleep(0.01)
            continue

        x_mm, y_mm = robotWorld

        # Orientation from top edge of the marker, in [0, 2π)
        theta = None
        if ROBOT_ID in cornerMap:
            rCorners = cornerMap[ROBOT_ID].astype(float)
            topMidPx = (rCorners[0] + rCorners[1]) / 2.0
            topMidWorld = pixelToWorld(topMidPx, VISION_H)
            if topMidWorld is not None:
                dx = topMidWorld[0] - x_mm
                dy = topMidWorld[1] - y_mm
                theta = math.atan2(dy, dx)

        if showDisplay:
            dispFrame = frame.copy()
            drawRobot(dispFrame, centerMap[ROBOT_ID], cornerMap.get(ROBOT_ID))
            cv2.imshow("Robot Tracking", dispFrame)
            cv2.waitKey(1)

        return x_mm, y_mm, theta, True


def stopVision():
    """
    Cleanly stop the cached camera and close any OpenCV windows.
    """
    global VISION_CAMERA
    if VISION_CAMERA is not None:
        VISION_CAMERA.stop()
        VISION_CAMERA = None
    cv2.destroyAllWindows()


# MAIN TEST HARNESS
# ============================================================

if __name__ == "__main__":
    print("=" * 40)

    # Heavy init once: compute homography, obstacles, etc.
    coords, theta = getVisionCoords(timeout=None, showDisplay=True)

    if coords.size == 0:
        print("No coordinates found or timeout occurred")
    else:
        print("Returned coordinates array:")
        print(coords)
        print("Robot theta (world frame, rad):", theta)

        # Example of fast position refresh (blocks until robot detected):
        x, y, th = getRobotPositionMm(showDisplay=False)
        print("Fast robot position (mm):", x, y, "theta:", th)

    stopVision()