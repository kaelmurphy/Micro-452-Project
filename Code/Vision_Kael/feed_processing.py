import cv2
import numpy as np
from aruco_utils import detectAruco, buildOperatingZone
from draw_utils import drawOperatingZone, drawRobotGoal
from coord_utils import worldToZone, robotWorldPose, smoothTuple, smoothAngle, asXy
from obstacle import Obstacle

def detectEdges(frame, low=30, high=100, blur=3):
    '''
    detect edges using Canny with CLAHE preprocessing
    '''
    # convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # apply contrast-limited adaptive histogram equalization
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    # apply median blur to reduce noise
    if blur > 0:
        gray = cv2.medianBlur(gray, blur | 1)
    # detect edges using Canny
    return cv2.Canny(gray, low, high)

def getOperatingState(state):
    '''
    extract clean output dictionary with all coordinates in mm and angles in degrees for use by path planning module
    '''
    # initialize output with zone corners and obstacles
    output = {'zoneCorners': state.get('zoneCorners'), 'obstacles': state.get('obstacles', []), 
              'goal': None, 'robot': None}
    # add goal if detected
    goal = state.get('goal')
    if goal:
        output['goal'] = {'x': goal[0], 'y': goal[1]}
    # add robot if detected
    robot = state.get('robot')
    theta = state.get('robotTheta')
    if robot and theta is not None:
        output['robot'] = {'x': robot[0], 'y': robot[1], 'theta': theta}
    return output

def detectAndDrawObstacles(canvas, edges, pixelToWorld, zoneCornersMm, minArea=500, maxVertices=10):
    '''
    detect obstacles from edges, draw on canvas, return Obstacle objects with vertices in mm
    '''
    if not zoneCornersMm:
        return canvas, [], (0, 0)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # zone dimensions: corners = [tl, tr, br, bl]
    bottomLen = np.linalg.norm(np.array(zoneCornersMm[2]) - np.array(zoneCornersMm[3]))  # BR to BL
    leftLen = np.linalg.norm(np.array(zoneCornersMm[0]) - np.array(zoneCornersMm[3]))  # TL to BL
    obstacles = []
    for i, contour in enumerate(contours):
        # filter by area and vertex count
        if cv2.contourArea(contour) < minArea or len(contour) > maxVertices:
            continue
        # approximate contour to polygon
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) > maxVertices:
            continue
        obstacle = Obstacle(i)
        pixelPts = []
        # convert each vertex to mm coordinates
        for pt in approx:
            px, py = pt[0]
            pixelPts.append([px, py])
            # convert to zone coordinates then scale by zone dimensions
            zonePt = worldToZone(pixelToWorld((px, py)), {'corners': zoneCornersMm})
            if zonePt and -0.1 <= zonePt[0] <= 1.1 and -0.1 <= zonePt[1] <= 1.1:
                obstacle.addVertex(zonePt[0] * bottomLen, zonePt[1] * leftLen)
        # only keep obstacles with at least 3 vertices
        if len(obstacle.getVertices()) >= 3:
            obstacles.append(obstacle)
            # draw obstacle on canvas
            pts = np.array(pixelPts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts], True, (255, 0, 255), 2, cv2.LINE_AA)
            # add semi-transparent fill
            overlay = canvas.copy()
            cv2.fillPoly(overlay, [pts], (255, 0, 255))
            cv2.addWeighted(overlay, 0.2, canvas, 0.8, 0, canvas)
    return canvas, obstacles, (bottomLen, leftLen)

def createCanvasAndState(frame, robotId=8, goalId=9, edgeParams={'low': 25, 'high': 80, 'blur': 3}):
    '''
    main vision pipeline, returns canvas with overlays and state dict with coordinates in mm
    '''
    # detect edges for obstacle detection
    edges = detectEdges(frame, **edgeParams)
    # unique key for smoothing this robot/goal pair
    smoothKey = f"r{robotId}_g{goalId}"
    # create white canvas and draw edges in black
    canvas = np.full_like(frame, 255)
    canvas[edges > 0] = (0, 0, 0)
    # detect ArUco markers
    _, centers, cornersMap, _ = detectAruco(frame, draw=False)
    # build operating zone from corner markers
    zone = buildOperatingZone(centers)
    # draw zone boundary on canvas
    canvas = drawOperatingZone(canvas, zone)
    frameH, frameW = frame.shape[:2]
    # calibrate scale from robot's 50mm orientation line
    pixelsPerMm = max(frameW, frameH) / 200.0  # fallback
    if cornersMap and robotId in cornersMap:
        arr = cornersMap[robotId].astype(float)
        # compute marker center
        cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
        # get top edge midpoint
        topMid = (arr[0] + arr[1]) / 2
        # measure distance from center to top edge (should be 25mm)
        lineLen = np.hypot(topMid[0] - cx, topMid[1] - cy)
        if lineLen > 0:
            pixelsPerMm = lineLen / 50.0  # 50mm = full marker height
    # conversion function: pixel coords to world coords in mm with bottom-left origin
    pixelToWorld = lambda pt: (pt[0] / pixelsPerMm, (frameH - 1 - pt[1]) / pixelsPerMm)
    # convert zone corners to mm
    zoneCornersMm = None
    zoneDims = (0, 0)
    if zone and zone.get('corners'):
        zoneCornersMm = [pixelToWorld(c) for c in zone['corners']]
    # detect obstacles and get zone dimensions
    canvas, obstacles, zoneDims = detectAndDrawObstacles(canvas, edges, pixelToWorld, zoneCornersMm)
    # draw robot and goal markers
    canvas = drawRobotGoal(canvas, cornersMap if cornersMap else centers, robotId, goalId)

    # process goal position
    goalZone = None
    if goalId in centers and zoneCornersMm and zoneDims[0] > 0:
        # convert goal to zone coordinates then scale to mm
        gz = worldToZone(pixelToWorld(centers[goalId]), {'corners': zoneCornersMm})
        if gz:
            goalZone = (gz[0] * zoneDims[0], gz[1] * zoneDims[1])
    # process robot position and orientation
    robotZone = None
    robotThetaZone = None
    rCx, rCy, _ = robotWorldPose(centers, cornersMap, robotId)
    if rCx and zoneCornersMm and cornersMap and robotId in cornersMap:
        # convert robot center to world coordinates
        robotWorld = pixelToWorld((rCx, rCy))
        arr = cornersMap[robotId].astype(float)
        # get top edge midpoint in world coordinates
        topMid = (arr[0] + arr[1]) / 2
        topMidWorld = pixelToWorld(topMid)
        # compute robot orientation in world frame
        robotThetaWorld = np.arctan2(topMidWorld[1] - robotWorld[1], topMidWorld[0] - robotWorld[0])
        # convert robot position to zone coordinates
        rz = worldToZone(robotWorld, {'corners': zoneCornersMm})
        if rz and zoneDims[0] > 0:
            robotZone = (rz[0] * zoneDims[0], rz[1] * zoneDims[1])
            # theta relative to zone bottom edge (0° = right along bottom edge)
            edgeAngle = np.arctan2(zoneCornersMm[2][1] - zoneCornersMm[3][1], 
                                   zoneCornersMm[2][0] - zoneCornersMm[3][0])
            robotThetaZone = np.degrees(robotThetaWorld - edgeAngle)
            robotThetaZone = ((robotThetaZone + 180) % 360) - 180  # normalize to [-180, 180]
    # build state dictionary with smoothed coordinates
    state = {'zoneCorners': zoneCornersMm, 'goal': smoothTuple(smoothKey, 'goal', goalZone),
             'robot': smoothTuple(smoothKey, 'robot', robotZone),
             'robotTheta': smoothAngle(smoothKey, 'robotTheta', robotThetaZone),
             'obstacles': obstacles}

    # draw status text on canvas
    lines = []
    if state['zoneCorners']:
        tl, tr, br, bl = state['zoneCorners']
        lines.append(f"BL:({bl[0]:.1f},{bl[1]:.1f}) BR:({br[0]:.1f},{br[1]:.1f})")
        lines.append(f"TL:({tl[0]:.1f},{tl[1]:.1f}) TR:({tr[0]:.1f},{tr[1]:.1f})")
    if asXy(state['goal']):
        gx, gy = state['goal']
        lines.append(f"Goal: x={gx:.1f}mm y={gy:.1f}mm")
    if asXy(state['robot']):
        rx, ry = state['robot']
        theta = f"{state['robotTheta']:.1f}°" if state['robotTheta'] is not None else "n/a"
        lines.append(f"Robot: x={rx:.1f}mm y={ry:.1f}mm θ={theta}")
    # draw text with black outline and white fill
    for i, txt in enumerate(reversed(lines)):
        y = canvas.shape[0] - 8 - (i * 20)
        cv2.putText(canvas, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(canvas, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas, state