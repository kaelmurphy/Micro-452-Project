import cv2
import numpy as np
from aruco_utils import detectAruco, buildOperatingZone
from draw_utils import drawOperatingZone, drawRobotGoal
from coord_utils import worldToZone, robotWorldPose, _smooth_tuple, _smooth_angle, _as_xy

def toGray(frame):
    # convert frame to grayscale
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def detectEdges(frame, low=30, high=100, blur=3):
    # convert to grayscale
    g = toGray(frame)
    # apply contrast to make edges more visible
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    g = clahe.apply(g)
    # median blur to clean up noise
    if blur and blur > 0:
        k = blur | 1
        g = cv2.medianBlur(g, k)
    # canny edge detection
    return cv2.Canny(g, low, high)

def createCanvasAndState(frame, robotId=8, goalId=9,
                         edgeParams=dict(low=25, high=80, blur=3)):
    """
    produces white canvas with black edges and colored overlays,
    returns a state dict for path planning.

    returns: (canvas_bgr, state)
    state contains keys: zone, zoneWorld, goalWorld, goalZone, robotWorld,
    robotZone, robotThetaWorld, robotThetaZone, ids
    """
    # detect edges
    edges = detectEdges(frame, **edgeParams)

    # smoothing key for this camera/robot/goal combination
    _smoothKey = f"r{robotId}_g{goalId}"

    # create white canvas and draw black edges
    canvas = np.full_like(frame, 255)
    canvas[edges > 0] = (0, 0, 0)

    # detect aruco markers
    ids, centers, cornersMap, _ = detectAruco(frame, dictName="DICT_4X4_50", draw=False)

    # build operating zone and draw
    zone = buildOperatingZone(centers)
    canvas = drawOperatingZone(canvas, zone, color=(0, 255, 255))

    # draw robot and goal overlays (prefer cornersMap so outlines match)
    drawSource = cornersMap if cornersMap else centers
    canvas = drawRobotGoal(canvas, drawSource, robotId=robotId, goalIds=[goalId])

    # compute zone_world (corners and centroid)
    # map pixel coordinates into a world coordinate system where
    # image top-left -> (0,0) and image bottom-right -> (10000,10000).
    frame_h, frame_w = frame.shape[:2]
    if frame_w <= 1 or frame_h <= 1:
        sx = sy = 1.0
    else:
        sx = 10000.0 / float(frame_w - 1)
        sy = 10000.0 / float(frame_h - 1)

    def pixelToWorld(pt):
        """
        convert an (x,y) pixel tuple to world coords (xw, yw) in [0,10000].
        """
        x, y = float(pt[0]), float(pt[1])
        return (x * sx, y * sy)

    zoneWorld = None
    if zone and zone.get('corners'):
        # convert each corner to world coords
        worldCorners = [pixelToWorld(c) for c in zone['corners']]
        wc = np.array(worldCorners, dtype=float)
        zCx = float(wc[:, 0].mean())
        zCy = float(wc[:, 1].mean())
        zoneWorld = {
            'corners': worldCorners,
            'center': (zCx, zCy)
        }

    # goal world and in-zone coords
    goalWorld = None
    goalZone = None
    if goalId in centers:
        # map goal pixel center to world coords
        goalWorld = pixelToWorld(centers[goalId])
        if zoneWorld:
            # worldToZone expects zone corners in the same coord space as the point
            gz = worldToZone(goalWorld, {'corners': zoneWorld['corners']})
            # scale to 0-10000 where tl->(0,0) and br->(10000,10000)
            if gz is not None:
                goalZone = (float(gz[0]) * 10000.0, float(gz[1]) * 10000.0)
            else:
                goalZone = None

    # apply smoothing to goal/world coords (store smoothed copies)
    goalWorldSmoothed = _smooth_tuple(_smoothKey, 'goalWorld', goalWorld)
    goalZoneSmoothed = _smooth_tuple(_smoothKey, 'goalZone', goalZone)

    # robot world and orientation
    robotWorld = None
    robotZone = None
    robotThetaWorld = None
    robotThetaZone = None
    rCx, rCy, rTheta = robotWorldPose(centers, cornersMap=cornersMap, robotId=robotId)
    if rCx is not None:
        # map robot center to world coords
        robotWorld = pixelToWorld((rCx, rCy))

        # compute theta in world coordinates: if corners available, use top-middle
        if cornersMap and robotId in cornersMap:
            arr = np.asarray(cornersMap[robotId], dtype=float)
            topMidPx = np.array([(arr[0,0] + arr[1,0]) / 2.0, (arr[0,1] + arr[1,1]) / 2.0])
            topMidWorld = pixelToWorld((topMidPx[0], topMidPx[1]))
            cxw, cyw = robotWorld
            vecWorld = np.array([topMidWorld[0] - cxw, topMidWorld[1] - cyw], dtype=float)
            robotThetaWorld = float(np.arctan2(vecWorld[1], vecWorld[0]))
        else:
            # fallback: if only rTheta (pixel-space) available, convert using scale factors
            if rTheta is not None:
                ux = np.cos(rTheta) * sx
                uy = np.sin(rTheta) * sy
                robotThetaWorld = float(np.arctan2(uy, ux))
            else:
                robotThetaWorld = None

        # compute robot zone-local coords using world coords
        if zoneWorld:
            rz = worldToZone(robotWorld, {'corners': zoneWorld['corners']})
            if rz is not None:
                robotZone = (float(rz[0]) * 10000.0, float(rz[1]) * 10000.0)
            else:
                robotZone = None
            # compute orientation relative to the bottom edge of the zone
            if robotThetaWorld is not None:
                try:
                    # zoneWorld corners are [tl, tr, br, bl]
                    tl_w, tr_w, br_w, bl_w = zoneWorld['corners']
                    tl_w = np.array(tl_w, dtype=float)
                    tr_w = np.array(tr_w, dtype=float)
                    br_w = np.array(br_w, dtype=float)
                    bl_w = np.array(bl_w, dtype=float)
                    # bottom edge vector from left-bottom (bl) -> right-bottom (br)
                    edgeVec = br_w - bl_w
                    edgeAngle = float(np.arctan2(edgeVec[1], edgeVec[0]))
                    # relative angle = robot heading - edge angle, normalized to [-pi, pi]
                    rel = robotThetaWorld - edgeAngle
                    # normalize
                    rel = (rel + np.pi) % (2 * np.pi) - np.pi
                    robotThetaZone = float(rel)
                except Exception:
                    robotThetaZone = None

    # apply smoothing to robot/world coords and angle
    robotWorldSmoothed = _smooth_tuple(_smoothKey, 'robotWorld', robotWorld)
    robotZoneSmoothed = _smooth_tuple(_smoothKey, 'robotZone', robotZone)
    robotThetaZoneSmoothed = _smooth_angle(_smoothKey, 'robotThetaZone', robotThetaZone)

    state = {
        'zone': zone,
        'zoneWorld': zoneWorld,
        'goalWorld': goalWorld,
        'goalZone': goalZone,
        'goalWorldSmoothed': goalWorldSmoothed,
        'goalZoneSmoothed': goalZoneSmoothed,
        'robotWorld': robotWorld,
        'robotZone': robotZone,
        'robotWorldSmoothed': robotWorldSmoothed,
        'robotZoneSmoothed': robotZoneSmoothed,
        'robotThetaWorld': robotThetaWorld,
        'robotThetaZone': robotThetaZone,
        'robotThetaZoneSmoothed': robotThetaZoneSmoothed,
        'ids': ids,
        'centers': centers,
        'cornersMap': cornersMap
    }

    # draw status text in bottom-left corner: show goal x,y and robot x,y,theta
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.7
    thickness = 2
    lineHeight = int(20 * fontScale) + 6
    pad = 8
    canvas_h = canvas.shape[0]
    # start baseline near bottom-left and draw lines upwards
    lines = []
    # goal coords
    gzs = _as_xy(state.get('goalZoneSmoothed'))
    gw = _as_xy(state.get('goalWorldSmoothed'))
    if gzs is not None:
        gx, gy = gzs
        lines.append(f"Goal: x={int(round(gx))} y={int(round(gy))}")
    elif gw is not None:
        gx, gy = gw
        lines.append(f"Goal(world): x={int(round(gx))} y={int(round(gy))}")
    else:
        lines.append("Goal: n/a")

    # robot coords and angle
    rzs = _as_xy(state.get('robotZoneSmoothed'))
    rws = _as_xy(state.get('robotWorldSmoothed'))
    rtheta_s = state.get('robotThetaZoneSmoothed')
    if rzs is not None:
        rx, ry = rzs
        if rtheta_s is not None:
            deg = rtheta_s * 180.0 / np.pi
            lines.append(f"Robot: x={int(round(rx))} y={int(round(ry))} theta={deg:.1f}deg")
        else:
            lines.append(f"Robot: x={int(round(rx))} y={int(round(ry))} theta=n/a")
    elif rws is not None:
        rx, ry = rws
        rthetaw = state.get('robotThetaWorld')
        if rthetaw is not None:
            deg = rthetaw * 180.0 / np.pi
            lines.append(f"Robot(world): x={int(round(rx))} y={int(round(ry))} theta={deg:.1f}deg")
        else:
            lines.append(f"Robot(world): x={int(round(rx))} y={int(round(ry))} theta=n/a")
    else:
        lines.append("Robot: n/a")

    # draw each line from bottom up
    for i, txt in enumerate(reversed(lines)):
        y = canvas_h - pad - (i * lineHeight)
        cv2.putText(canvas, txt, (pad, y), font, fontScale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(canvas, txt, (pad, y), font, fontScale, (255, 255, 255), thickness, cv2.LINE_AA)

    return canvas, state