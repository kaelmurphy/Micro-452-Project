import cv2
import threading
import numpy as np
import math
import time

def detectAruco(frame, dictName="DICT_4X4_50"):
    if not hasattr(cv2, "aruco"):
        return [], {}, {}, frame

    dict_attr = getattr(cv2.aruco, dictName, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
    detector = cv2.aruco.ArucoDetector(dictionary)

    # Use full-resolution grayscale image for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Optionally, apply contrast enhancement if needed (commented out for now)
    # gray = cv2.equalizeHist(gray)
    corners, ids, _ = detector.detectMarkers(gray)

    centers = {}
    cornersMap = {}
    idList = []

    if ids is not None:
        idList = ids.flatten().tolist()
        for i, marker_id in enumerate(idList):
            pts = corners[i][0]  # No scaling needed
            cornersMap[marker_id] = pts.astype(int)
            center = pts.mean(axis=0).astype(int)
            centers[marker_id] = tuple(center)

    return idList, centers, cornersMap, frame  # Skip annotated frame for speed

def buildOperatingZone(centers):
    required = [0, 1, 2, 3]
    missing = [i for i in required if i not in centers]
    # return corners in order [bl, br, tr, tl] as expected by feed_processing
    corner_order = [3, 2, 1, 0]  # [bl, br, tr, tl]
    return {
        'isComplete': len(missing) == 0,
        'missing': missing,
        'corners': [centers[i] for i in corner_order if i in centers]
    }

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




def asXy(val):
    if val is None:
        return None
    try:
        return (float(val[0]), float(val[1]))
    except:
        return None

def robotWorldPose(centers, cornersMap, robotId):
    if robotId not in centers:
        return (None, None, None)
    
    try:
        cx, cy = int(centers[robotId][0]), int(centers[robotId][1])
    except:
        return (None, None, None)
    
    if robotId in cornersMap:
        corners = cornersMap[robotId].astype(float)
        topMid = (corners[0] + corners[1]) / 2.0
        dx = topMid[0] - cx
        dy = topMid[1] - cy
        theta = float(np.arctan2(-dy, dx))
        return (cx, cy, theta)
    
    return (cx, cy, None)



def drawOperatingZone(frame, zone, color=(0, 255, 255)):
    if not zone.get('corners'):
        return frame
    pts = np.array(zone['corners'], dtype=np.int32)
    cv2.polylines(frame, [pts], True, color, 1)  # Thin line only
    return frame

def drawCoordinateSystem(canvas, origin, axis_length=40):
    """Draw coordinate system axes at the origin (ultra-minimal)."""
    origin_px = (int(origin[0]), int(origin[1]))
    x_end = (origin_px[0] + axis_length, origin_px[1])
    y_end = (origin_px[0], origin_px[1] - axis_length)
    
    # Ultra-thin axes
    cv2.line(canvas, origin_px, x_end, (0, 0, 255), 1)  # X-axis (red)
    cv2.line(canvas, origin_px, y_end, (0, 255, 0), 1)  # Y-axis (green)
    cv2.circle(canvas, origin_px, 3, (0, 0, 0), -1)  # Tiny origin point

def drawRobotMarker(canvas, center, corners=None):
    """Draw robot marker with orientation arrow."""
    robot_center = (int(center[0]), int(center[1]))
    cv2.circle(canvas, robot_center, 6, (255, 0, 0), -1)  # Tiny blue circle
    
    # Draw shorter orientation arrow if corners available
    if corners is not None:
        top_mid = ((corners[0] + corners[1]) / 2).astype(int)
        # Make arrow line shorter - half of the previous length
        direction = top_mid - robot_center
        extended_end = robot_center + (direction * 0.9).astype(int)  # 0.9x length
        cv2.arrowedLine(canvas, robot_center, tuple(extended_end), (255, 150, 150), 2)

def drawGoalMarker(canvas, center):
    """Draw goal marker - just green circle in center."""
    goal_center = (int(center[0]), int(center[1]))
    cv2.circle(canvas, goal_center, 8, (0, 255, 0), -1)  # Green circle, slightly bigger for visibility


# Configuration constants
ROBOT_ID = 8
GOAL_ID = 9
ORIGIN_ID = 3
STABILITY_FRAMES = 30
STABILITY_THRESHOLD = 6.0
MARKER_SIZE_MM = 96.5

# Global state
stabilityBuffer = []
isStable = stableScreenshot = coordinatesCaptured = False

def resetStability():
    """Reset stability tracking state."""
    global isStable, stableScreenshot, coordinatesCaptured
    isStable = stableScreenshot = coordinatesCaptured = False
    stabilityBuffer.clear()
    print("\n=== STABILITY RESET ===")

def convertPixelsToWorldCoords(pixelCoords, originPixel, cornersMap, markerSizeMm=MARKER_SIZE_MM):
    """Convert pixel coordinates to world coordinates."""
    pixelsPerMm = calculateScale(cornersMap, markerSizeMm)
    
    # Handle single coordinate pair
    if isinstance(pixelCoords, (tuple, list)) and len(pixelCoords) == 2 and isinstance(pixelCoords[0], (int, float, np.integer)):
        x_mm = (float(pixelCoords[0]) - float(originPixel[0])) / pixelsPerMm
        y_mm = (float(originPixel[1]) - float(pixelCoords[1])) / pixelsPerMm
        return (x_mm, y_mm)
    
    # Handle list of coordinate pairs
    result = []
    for coord in pixelCoords:
        try:
            if isinstance(coord, (tuple, list)) and len(coord) >= 2:
                px, py = float(coord[0]), float(coord[1])
            elif isinstance(coord, np.ndarray) and coord.size >= 2:
                flat = coord.flatten()
                px, py = float(flat[0]), float(flat[1])
            else:
                continue
                
            x_mm = (px - float(originPixel[0])) / pixelsPerMm
            y_mm = (float(originPixel[1]) - py) / pixelsPerMm
            result.append((x_mm, y_mm))
        except (ValueError, TypeError, IndexError):
            continue
    return result

def calculateScale(cornersMap, markerSizeMm=MARKER_SIZE_MM):
    """Calculate pixels per mm using median of all marker edges."""
    if not cornersMap:
        return 1.0
    
    edgeLengths = []
    for corners in cornersMap.values():
        if corners is not None and len(corners) == 4:
            for i in range(4):
                edgeLength = np.linalg.norm(corners[(i + 1) % 4] - corners[i])
                if edgeLength > 0:
                    edgeLengths.append(edgeLength)
    
    return np.median(edgeLengths) / markerSizeMm if edgeLengths else 1.0

def checkStability(state):
    """Check if robot and goal positions are stable across frames."""
    robot, goal, theta = state.get('start'), state.get('goal'), state.get('robotTheta')
    if not (robot and goal and theta is not None):
        stabilityBuffer.clear()
        return False
    
    currentState = {'start': robot, 'goal': goal, 'robotTheta': theta}
    
    # Check movement against previous frame
    if stabilityBuffer:
        last = stabilityBuffer[-1]
        if (np.linalg.norm(np.array(robot) - np.array(last['start'])) > STABILITY_THRESHOLD or 
            np.linalg.norm(np.array(goal) - np.array(last['goal'])) > STABILITY_THRESHOLD or 
            abs(theta - last['robotTheta']) > 5.0):
            stabilityBuffer.clear()
    
    stabilityBuffer.append(currentState)
    return len(stabilityBuffer) >= STABILITY_FRAMES

def _drawCoordinateAxes(canvas, originPx, axisLength=100):
    """Draw coordinate system axes at origin."""
    xEnd = (int(originPx[0] + axisLength), int(originPx[1]))
    yEnd = (int(originPx[0]), int(originPx[1] - axisLength))
    
    cv2.arrowedLine(canvas, (int(originPx[0]), int(originPx[1])), xEnd, (0, 0, 255), 3)
    cv2.arrowedLine(canvas, (int(originPx[0]), int(originPx[1])), yEnd, (0, 255, 0), 3)
    cv2.putText(canvas, "X", (xEnd[0] + 5, xEnd[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(canvas, "Y", (yEnd[0] - 15, yEnd[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.circle(canvas, (int(originPx[0]), int(originPx[1])), 8, (255, 255, 255), -1)

def _drawMarkers(canvas, centers, cornersMap):
    """Draw robot and goal markers with orientation."""
    # Draw robot
    if ROBOT_ID in centers:
        robotCenter = (int(centers[ROBOT_ID][0]), int(centers[ROBOT_ID][1]))
        cv2.circle(canvas, robotCenter, 8, (255, 0, 0), -1)
        cv2.putText(canvas, "ROBOT", (robotCenter[0] + 15, robotCenter[1] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        if ROBOT_ID in cornersMap:
            corners = cornersMap[ROBOT_ID]
            topMid = ((corners[0] + corners[1]) / 2).astype(int)
            cv2.arrowedLine(canvas, robotCenter, tuple(topMid), (255, 150, 150), 3)
    
    # Draw goal
    if GOAL_ID in centers:
        goalCenter = (int(centers[GOAL_ID][0]), int(centers[GOAL_ID][1]))
        cv2.circle(canvas, goalCenter, 8, (0, 255, 0), -1)
        cv2.putText(canvas, "GOAL", (goalCenter[0] + 15, goalCenter[1] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

def _handleStabilityAndUI(canvas, state, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone):
    """Handle stability detection and UI rendering."""
    global isStable, stableScreenshot, stabilityBuffer
    
    statusLines = []
    
    if not isStable:
        if checkStability(state):
            isStable = True
            stableScreenshot = canvas.copy()
            _addStableLabels(stableScreenshot, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone)
            statusLines.append("SYSTEM LOCKED - COORDINATES STABLE")
        else:
            progress = len(stabilityBuffer) / STABILITY_FRAMES * 100
            statusLines.append(f"CALIBRATING... {progress:.0f}% ({len(stabilityBuffer)}/{STABILITY_FRAMES} frames)")
    else:
        statusLines.extend(["SYSTEM LOCKED - COORDINATES STABLE", "Press 'R' to recalibrate"])
    
    # Add system status
    robotStatus = "FOUND" if ROBOT_ID in centers else "NOT FOUND"
    goalStatus = "FOUND" if GOAL_ID in centers else "NOT FOUND"
    statusLines.extend([f"Robot ID {ROBOT_ID}: {robotStatus}", f"Goal ID {GOAL_ID}: {goalStatus}"])
    
    # Add coordinate info
    if zoneCornersMm and len(zoneCornersMm) == 4:
        bl, br, tr, tl = zoneCornersMm
        statusLines.extend([
            f"Zone (mm) - BL:({bl[0]:.1f},{bl[1]:.1f}) BR:({br[0]:.1f},{br[1]:.1f})",
            f"Zone (mm) - TL:({tl[0]:.1f},{tl[1]:.1f}) TR:({tr[0]:.1f},{tr[1]:.1f})"
        ])
    
    if goalZone:
        statusLines.append(f"Goal: x={goalZone[0]:.1f}mm y={goalZone[1]:.1f}mm")
    
    if robotZone:
        theta = f"{robotThetaZone:.1f}°" if robotThetaZone is not None else "n/a"
        statusLines.append(f"Robot: x={robotZone[0]:.1f}mm y={robotZone[1]:.1f}mm θ={theta}")
    
    # Draw status text with shadow effect
    _drawStatusText(canvas, statusLines)
    return canvas

def _addStableLabels(canvas, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone):
    """Add coordinate labels to stable screenshot."""
    if ORIGIN_ID in centers and zoneCornersMm and len(zoneCornersMm) == 4:
        bl, br, tr, tl = zoneCornersMm
        cornerData = [(ORIGIN_ID, (0.0, 0.0), "BL"), (2, br, "BR"), (1, tr, "TR"), (0, tl, "TL")]
        
        for markerId, coords, label in cornerData:
            if markerId in centers:
                center = centers[markerId]
                text = f"{label} = ({coords[0]:.1f}, {coords[1]:.1f})"
                _drawTextWithShadow(canvas, text, (int(center[0]) + 13, int(center[1]) + 25), (255, 255, 255))
    
    if robotZone and ROBOT_ID in centers:
        center = centers[ROBOT_ID]
        robotText = f"ROBOT = ({robotZone[0]:.1f}, {robotZone[1]:.1f})"
        if robotThetaZone is not None:
            robotText += f", {robotThetaZone:.1f}°"
        _drawTextWithShadow(canvas, robotText, (int(center[0]) + 13, int(center[1]) + 40), (255, 200, 200))
    
    if goalZone and GOAL_ID in centers:
        center = centers[GOAL_ID]
        goalText = f"GOAL = ({goalZone[0]:.1f}, {goalZone[1]:.1f})"
        _drawTextWithShadow(canvas, goalText, (int(center[0]) + 13, int(center[1]) + 40), (0, 255, 0))

def _drawTextWithShadow(canvas, text, position, color, shadowOffset=(2, 2)):
    """Draw text with shadow for better visibility."""
    shadowPos = (position[0] + shadowOffset[0], position[1] + shadowOffset[1])
    cv2.putText(canvas, text, shadowPos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
    cv2.putText(canvas, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def _printAllCoordinates(state, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone, obstacles, cornersMap, originPx):
    """Print all coordinates when system becomes stable."""
    print("\n" + "="*60)
    print("         COORDINATE CAPTURE - STABLE IMAGE DETECTED")
    print("="*60)
    
    # Zone corners
    if zoneCornersMm and len(zoneCornersMm) == 4:
        bl, br, tr, tl = zoneCornersMm
        print(f"\nOPERATING ZONE CORNERS (mm):")
        print(f"  Bottom-Left:  ({bl[0]:.1f}, {bl[1]:.1f})")
        print(f"  Bottom-Right: ({br[0]:.1f}, {br[1]:.1f})")
        print(f"  Top-Right:    ({tr[0]:.1f}, {tr[1]:.1f})")
        print(f"  Top-Left:     ({tl[0]:.1f}, {tl[1]:.1f})")
    
    # Robot position
    if robotZone:
        theta_str = f"{robotThetaZone:.1f}°" if robotThetaZone is not None else "n/a"
        print(f"\nROBOT POSITION (mm):")
        print(f"  X: {robotZone[0]:.1f}")
        print(f"  Y: {robotZone[1]:.1f}")
        print(f"  Θ: {theta_str}")
    
    # Goal position
    if goalZone:
        print(f"\nGOAL POSITION (mm):")
        print(f"  X: {goalZone[0]:.1f}")
        print(f"  Y: {goalZone[1]:.1f}")
    
    # Obstacles - use centralized coordinate conversion
    if obstacles:
        print(f"\nOBSTACLES DETECTED: {len(obstacles)}")
        for i, obstacle in enumerate(obstacles, 1):
            # Use centralized coordinate conversion
            vertices = obstacle.getVerticesWorldCoords(cornersMap, originPx)
            print(f"\n  Obstacle {i}:")
            print(f"    Area: {obstacle.area:.0f} pixels")
            print(f"    Vertices (mm, CCW from bottom-left):")
            for j, (x, y) in enumerate(vertices):
                print(f"      V{j}: ({x:.1f}, {y:.1f})")
    else:
        print("\nOBSTACLES: None detected")
    
    print("\n" + "="*60)
    print("Press 'R' to reset and capture new coordinates")
    print("="*60 + "\n")

def _drawStatusText(canvas, lines):
    """Draw status text at bottom of canvas."""
    for i, text in enumerate(reversed(lines)):
        y = canvas.shape[0] - 20 - (i * 25)
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

def createCanvasAndState(frame):
    """Ultra-lightweight main vision processing pipeline."""
    global stableScreenshot, isStable, coordinatesCaptured
    
    # Return frozen stable image if already captured
    if isStable and stableScreenshot is not None:
        return stableScreenshot, {}
    
    canvas = frame.copy()
    ids, centers, cornersMap, _ = detectAruco(frame)
    zone = buildOperatingZone(centers)
    
    # Early exit if no origin marker
    if ORIGIN_ID not in centers:
        return canvas, {}
    
    originPx = centers[ORIGIN_ID]
    zoneCornersMm = goalZone = robotZone = robotThetaZone = None
    
    if zone.get('isComplete'):
        zoneCornersMm = convertPixelsToWorldCoords(zone['corners'], originPx, cornersMap)
    
    if GOAL_ID in centers:
        goalZone = convertPixelsToWorldCoords(centers[GOAL_ID], originPx, cornersMap)
    
    rCx, rCy, robotTheta = robotWorldPose(centers, cornersMap, ROBOT_ID)
    if rCx is not None and rCy is not None:
        robotZone = convertPixelsToWorldCoords((rCx, rCy), originPx, cornersMap)
        if robotTheta is not None:
            robotThetaZone = np.degrees(robotTheta)
            if robotThetaZone < 0:
                robotThetaZone += 360
    
    # Obstacle detection
    obstacles = []
    if zone.get('isComplete') and not isStable:
        obstacles = detectObstacles(frame, zone, minArea=2500)
    
    # Minimal rendering
    if zone.get('isComplete'):
        canvas = drawOperatingZone(canvas, zone)
    
    if obstacles:
        canvas = drawObstacles(canvas, obstacles, cornersMap, originPx, False, False)
    


    drawCoordinateSystem(canvas, originPx)
    
    if ROBOT_ID in centers:
        drawRobotMarker(canvas, centers[ROBOT_ID], cornersMap.get(ROBOT_ID))
    if GOAL_ID in centers:
        drawGoalMarker(canvas, centers[GOAL_ID])
    
    # Build state
    state = {
        'goal': goalZone,
        'start': robotZone,
        'robotTheta': robotThetaZone,
        'obstacles': [obs.toDict() for obs in obstacles]
    }
    
    # Stability system
    if not isStable and checkStability(state):
        isStable = True
        stableScreenshot = canvas.copy()
        if not coordinatesCaptured:
            coordinatesCaptured = True
            _printAllCoordinates(state, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone, obstacles, cornersMap, originPx)
    
    # Status display
    stability_progress = len(stabilityBuffer) / STABILITY_FRAMES
    if isStable:
        cv2.putText(canvas, "STABLE", (canvas.shape[1]-120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(canvas, f"STABILIZING {int(stability_progress * 100)}%", (canvas.shape[1]-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    return canvas, state

def getOperatingState(state):
    """Convert internal state to structured output format."""
    output = {
        'zoneCorners': state.get('zoneCorners'),
        'obstacles': state.get('obstacles', []),
        'obstacleVertices': state.get('obstacleVertices', []),
        'goal': None,
        'start': None,
        'timestamp': cv2.getTickCount() / cv2.getTickFrequency()
    }
    
    if state.get('goal'):
        output['goal'] = {'x': state['goal'][0], 'y': state['goal'][1]}
    
    if state.get('start') and state.get('robotTheta') is not None:
        output['start'] = {
            'x': state['start'][0], 
            'y': state['start'][1], 
            'theta': state['robotTheta']
        }
    
    return output







# Global variables for temporal smoothing
contourHistory = []
HISTORY_SIZE = 3  # Increased for better tracking consistency

class Obstacle:
    def __init__(self, contour, area):
        self.contour = contour
        self.area = area
        self._vertices = self._worldVertices = None
    
    @property
    def vertices(self):
        if self._vertices is None:
            # Use approxPolyDP with higher epsilon for fewer, more prominent vertices
            peri = cv2.arcLength(self.contour, True)
            approx = cv2.approxPolyDP(self.contour, 0.02 * peri, True)
            # Round coordinates for stability
            self._vertices = [(int(round(p[0][0])), int(round(p[0][1]))) for p in approx]
        return self._vertices
    
    def getVerticesWorldCoords(self, cornersMap, origin):
        world_vertices = convertPixelsToWorldCoords(self.vertices, origin, cornersMap)
        return self._sortVerticesCCWFromBottomLeft(world_vertices)
    
    def _sortVerticesCCWFromBottomLeft(self, vertices):
        if len(vertices) < 3:
            return vertices
        
        cx = sum(v[0] for v in vertices) / len(vertices)
        cy = sum(v[1] for v in vertices) / len(vertices)
        
        def angle(vertex):
            angle = math.atan2(vertex[1] - cy, vertex[0] - cx)
            return angle + 2 * math.pi if angle < 0 else angle
        
        sorted_vertices = sorted(vertices, key=angle)
        bottom_left_idx = min(range(len(sorted_vertices)), 
                             key=lambda i: (sorted_vertices[i][1], sorted_vertices[i][0]))
        return sorted_vertices[bottom_left_idx:] + sorted_vertices[:bottom_left_idx]
    
    def toDict(self):
        return {'contour': self.contour.tolist(), 'area': self.area, 'vertices': self.vertices}

def detectObstacles(frame, zone, minArea=4000, maxArea=50000, colorRange=None):
    """
    Detect colored obstacles with optimized processing pipeline.
    """
    # Early exit if zone is incomplete
    if not zone.get('isComplete'):
        return []

    # Create zone mask to restrict detection to operating area only
    height, width = frame.shape[:2]
    zone_mask = np.zeros((height, width), dtype=np.uint8)
    zone_corners = zone.get('corners', [])
    if len(zone_corners) >= 3:
        zone_points = np.array(zone_corners, dtype=np.int32)
        cv2.fillPoly(zone_mask, [zone_points], 255)
    else:
        zone_mask[:] = 255

    # Use BGR filtering for new obstacle color (#3d7c80, BGR: 128, 124, 61)
    # BGR: (128, 124, 61) -- allow a range around this value
    lower_obstacle = np.array([80, 70, 20])     # BGR: tighter lower bound
    upper_obstacle = np.array([180, 170, 110])  # BGR: tighter upper bound
    mask = cv2.inRange(frame, lower_obstacle, upper_obstacle)

    # Apply zone mask
    mask = cv2.bitwise_and(mask, zone_mask)

    # Morphology for clean contours
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if minArea <= area <= maxArea and len(contour) >= 3:
            obstacles.append(Obstacle(contour, area))
    return obstacles


def processObstacles(obstacles, pixelsPerMm, origin, printInfo=False, verbose=False):
    """Process obstacles to extract vertices in world coordinates."""
    if not obstacles:
        return [], []
    
    obstacleData, allVertices = [], []
    
    for i, obstacle in enumerate(obstacles):
        worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
        if worldVertices:
            centroid = (sum(v[0] for v in worldVertices) / len(worldVertices),
                       sum(v[1] for v in worldVertices) / len(worldVertices))
        else:
            centroid = (0.0, 0.0)
        
        obstacleData.append({
            'id': i + 1,
            'vertices': worldVertices,
            'area': obstacle.area,
            'vertexCount': len(worldVertices),
            'centroid': centroid,
            'isCounterClockwise': _isCounterClockwise(worldVertices)
        })
        allVertices.extend(worldVertices)
    
    return obstacleData, allVertices

def _isCounterClockwise(vertices):
    """Check if vertices are in counter-clockwise order."""
    if len(vertices) < 3:
        return True
    
    signed_area = sum((vertices[(i + 1) % len(vertices)][0] - vertices[i][0]) * 
                     (vertices[(i + 1) % len(vertices)][1] + vertices[i][1]) 
                     for i in range(len(vertices)))
    return signed_area < 0

# Utility functions for polygon processing have been moved to class methods

def drawObstacles(canvas, obstacles, cornersMap, origin, showCoordinates=False, showVertexNumbers=False):
    """Minimal obstacle drawing."""
    if not obstacles:
        return canvas
    
    for obstacle in obstacles:
        contour = np.array(obstacle.contour, dtype=np.int32)
        cv2.fillPoly(canvas, [contour], (0, 0, 255))  # Solid red
        cv2.polylines(canvas, [contour], True, (0, 0, 255), 1)  # Red outline
    
    return canvas





def getCoordinatesFromVision(timeout=None, show_display=True):
    """
    Get coordinate data from the vision system.
    
    Args:
        timeout: Maximum time to wait for stability (None = wait forever)
        stabilityFrames: Number of consecutive stable frames required
        show_display: Whether to show live video feed (default: False)
    
    Returns:
        numpy array with columns [type, id, label, x, y]
        Empty array if failed or timeout
    """
    def coords_are_similar(arr1, arr2, tol=20.0):
        if arr1.shape != arr2.shape:
            return False
        # Only compare x_mm and y_mm columns (last two columns)
        xy1 = arr1[:, -2:].astype(float)
        xy2 = arr2[:, -2:].astype(float)
        return np.all(np.abs(xy1 - xy2) < tol)
    bufferLen = 3  # Require 3 stable frames for robust output
    coordsBuffer = []
    camera = None
    try:
        # Import inside function to avoid linting issues
        

        ROBOT_ID = 8
        GOAL_ID = 9
        ORIGIN_ID = 3

        camera = CameraStream(width=640, height=480, fps=30)
        camera.start()
        time.sleep(1)

        if show_display:
            cv2.namedWindow('Vision System', cv2.WINDOW_AUTOSIZE)

        startTime = time.time()
        while True:
            if timeout is not None:
                elapsed = time.time() - startTime
                if elapsed > timeout:
                    if camera is not None:
                        camera.stop()
                    return np.array([]).reshape(0, 5)

            frame = camera.read() if camera is not None else None
            if frame is None:
                time.sleep(2)
                continue

            display_frame = frame.copy()
            try:
                ids, centers, cornersMap, _ = detectAruco(frame)
                zone = buildOperatingZone(centers)
                originPx = centers.get(ORIGIN_ID)

                # Draw overlays if requested
                if show_display:
                    if zone.get('isComplete') and zone.get('corners'):
                        display_frame = drawOperatingZone(display_frame, zone, color=(0, 255, 255))
                    if ROBOT_ID in centers:
                        drawRobotMarker(display_frame, centers[ROBOT_ID], cornersMap.get(ROBOT_ID))
                    if GOAL_ID in centers:
                        drawGoalMarker(display_frame, centers[GOAL_ID])
                    if zone.get('isComplete'):
                        obstacles = detectObstacles(frame, zone, minArea=1000)
                        if obstacles:
                            display_frame = drawObstacles(display_frame, obstacles, cornersMap, originPx, False, False)
                    cv2.imshow('Vision System', display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        if camera is not None:
                            camera.stop()
                        cv2.destroyAllWindows()
                        return np.array([]).reshape(0, 5)

                # Always extract and return coordinates in mm
                coords = _extractCoordinatesFromFrame(frame, centers, cornersMap, zone, originPx)
                # Only start stabilization if all required elements are detected
                types = coords[:,0] if coords.size > 0 else []
                has_corners = np.sum(types == 'square') >= 4
                has_robot = np.any(types == 'start')
                has_goal = np.any(types == 'goal')
                has_obstacle = np.any(types == 'vertex')
                # For obstacles, compare by obstacle index and vertices
                def obstacles_are_similar(obs_arr1, obs_arr2, tol=50.0):
                    if len(obs_arr1) != len(obs_arr2):
                        return False
                    for row1, row2 in zip(obs_arr1, obs_arr2):
                        if row1[2] != row2[2] or row1[1] != row2[1]:
                            return False
                        x1, y1 = round(row1[3], 1), round(row1[4], 1)
                        x2, y2 = round(row2[3], 1), round(row2[4], 1)
                        if abs(x1 - x2) > tol or abs(y1 - y2) > tol:
                            return False
                    return True
                if coords.size > 0 and has_corners and has_robot and has_goal and has_obstacle:
                    # Separate obstacle rows for comparison
                    obstacle_rows = [row for row in coords if row[0] == 'vertex']
                    if len(coordsBuffer) == 0 or obstacles_are_similar(obstacle_rows, [row for row in coordsBuffer[-1] if row[0] == 'vertex']):
                        coordsBuffer.append(coords.copy())
                        if len(coordsBuffer) > bufferLen:
                            coordsBuffer = coordsBuffer[-bufferLen:]
                        # If all buffered arrays are similar, or if bufferLen reached and all elements are present, return immediately
                        if (len(coordsBuffer) == bufferLen and all(obstacles_are_similar(
                            [row for row in c if row[0] == 'vertex'],
                            [row for row in coordsBuffer[0] if row[0] == 'vertex']) for c in coordsBuffer)) or (len(coordsBuffer) == bufferLen and has_corners and has_robot and has_goal and has_obstacle):
                            if camera is not None:
                                camera.stop()
                            if show_display:
                                cv2.destroyAllWindows()
                            return coordsBuffer[0]
                    else:
                        coordsBuffer = [coords.copy()]
            except Exception as e:
                print(f"[DEBUG] Exception in main loop: {e}")
    except Exception:
        try:
            if camera is not None:
                camera.stop()
            if show_display:
                cv2.destroyAllWindows()
        except Exception:
            pass
        return np.array([]).reshape(0, 5)


def _statesAreSimilar(state1, state2, tolerance=10):
    """Check if two states are similar for stability detection."""
    if not state1 or not state2:
        return False
    
    if state1.get('start') and state2.get('start'):
        r1, r2 = state1['start'], state2['start']
        if abs(r1[0] - r2[0]) > tolerance or abs(r1[1] - r2[1]) > tolerance:
            return False
    elif state1.get('start') != state2.get('start'):
        return False
    
    if state1.get('goal') and state2.get('goal'):
        g1, g2 = state1['goal'], state2['goal']
        if abs(g1[0] - g2[0]) > tolerance or abs(g1[1] - g2[1]) > tolerance:
            return False
    elif state1.get('goal') != state2.get('goal'):
        return False
    
    if abs(state1.get('obstacles', 0) - state2.get('obstacles', 0)) > 0:
        return False
    
    return True

def _extractCoordinatesFromFrame(frame, centers, cornersMap, zone, originPx):
    """Extract all coordinates from a stable frame."""
    
    ROBOT_ID = 8
    GOAL_ID = 9
    coordinates_list = []
    
    try:
        if zone.get('isComplete'):
            corners_px = zone['corners']
            corner_labels = ['poly0'] * 4
            corner_names = ['3', '2', '1', '0']
            corners_world = convertPixelsToWorldCoords(corners_px, originPx, cornersMap)
            if isinstance(corners_world, (list, tuple, np.ndarray)) and len(corners_world) > 0 and hasattr(corners_world[0], '__iter__'):
                for (x, y), poly_id, label in zip(corners_world, corner_labels, corner_names):
                    coordinates_list.append(['square', poly_id, label, float(x), float(y)])
        if ROBOT_ID in centers:
            rCx, rCy, robotTheta = robotWorldPose(centers, cornersMap, ROBOT_ID)
            if rCx is not None and rCy is not None:
                robot_world = convertPixelsToWorldCoords((rCx, rCy), originPx, cornersMap)
                if isinstance(robot_world, tuple) and len(robot_world) == 2:
                    coordinates_list.append(['start', 'start_pt', 'Start', float(robot_world[0]), float(robot_world[1])])
        if GOAL_ID in centers:
            goal_world = convertPixelsToWorldCoords(centers[GOAL_ID], originPx, cornersMap)
            if isinstance(goal_world, tuple) and len(goal_world) == 2:
                coordinates_list.append(['goal', 'goal_pt', 'Goal', float(goal_world[0]), float(goal_world[1])])

        obstacles = detectObstacles(frame, zone, minArea=1000)
        if obstacles is None:
            obstacles = []
        for obs_idx, obstacle_obj in enumerate(obstacles):
            world_vertices = obstacle_obj.getVerticesWorldCoords(cornersMap, originPx)
            if isinstance(world_vertices, tuple) and len(world_vertices) == 2:
                world_vertices = [world_vertices]
            if isinstance(world_vertices, np.ndarray):
                world_vertices = world_vertices.tolist()
            poly_id = f"poly{obs_idx+1}"
            for v_idx, (x, y) in enumerate(world_vertices):
                coordinates_list.append([
                    'vertex',
                    poly_id,
                    str(v_idx + 1),
                    float(x),
                    float(y)
                ])

        return np.array(coordinates_list, dtype=object) if coordinates_list else np.array([]).reshape(0, 5)

    except Exception:
        print("No coordinates extracted from frame due to error.")
        return np.array([]).reshape(0, 5)


# Example usage and testing
if __name__ == "__main__":
    print("=" * 40)
    
    coordinates = getCoordinatesFromVision(timeout=None, stabilityFrames=20000)
    
    if coordinates.size == 0:
        print("No coordinates found or timeout occurred")
    else:
        print("Returned coordinates array:")
        print(coordinates)