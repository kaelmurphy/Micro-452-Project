import cv2
import numpy as np
from aruco_utils import detectAruco, buildOperatingZone
from draw_utils import drawOperatingZone
from coord_utils import robotWorldPose, asXy
from obstacle import detectObstacles, drawObstacles, processObstacles

# Configuration constants
ROBOT_ID = 8
GOAL_ID = 9
ORIGIN_ID = 3
STABILITY_FRAMES = 100
STABILITY_THRESHOLD = 5.0
MARKER_SIZE_MM = 100.0

# Global state
stabilityBuffer = []
stableScreenshot = None
isStable = False

def resetStability():
    """Reset stability tracking state."""
    global isStable, stableScreenshot, stabilityBuffer
    isStable = False
    stableScreenshot = None
    stabilityBuffer.clear()

def calculateScale(cornersMap, markerSizeMm=MARKER_SIZE_MM):
    """Calculate pixels per mm using median of all marker edges."""
    if not cornersMap:
        return 1.0
    
    edgeLengths = []
    for corners in cornersMap.values():
        if corners is not None and len(corners) == 4:
            # Calculate all 4 edge lengths for this marker
            for i in range(4):
                p1, p2 = corners[i], corners[(i + 1) % 4]
                edgeLength = np.linalg.norm(p2 - p1)
                if edgeLength > 0:  # Avoid division by zero
                    edgeLengths.append(edgeLength)
    
    if not edgeLengths:
        return 1.0
    
    # Use median for robustness against outliers
    medianEdgeLength = np.median(edgeLengths)
    return medianEdgeLength / markerSizeMm

def checkStability(state):
    """Check if robot and goal positions are stable across frames."""
    global stabilityBuffer
    
    # Validate required state
    robot = state.get('robot')
    goal = state.get('goal') 
    robotTheta = state.get('robotTheta')
    
    if not (robot and goal and robotTheta is not None):
        stabilityBuffer.clear()
        return False
    
    currentState = {
        'robot': robot, 
        'goal': goal, 
        'robotTheta': robotTheta
    }
    
    # Check movement against previous frame
    if stabilityBuffer:
        lastState = stabilityBuffer[-1]
        robotDiff = np.linalg.norm(np.array(robot) - np.array(lastState['robot']))
        goalDiff = np.linalg.norm(np.array(goal) - np.array(lastState['goal']))
        angleDiff = abs(robotTheta - lastState['robotTheta'])
        
        # Reset if significant movement detected
        if (robotDiff > STABILITY_THRESHOLD or 
            goalDiff > STABILITY_THRESHOLD or 
            angleDiff > 5.0):
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

def _drawStatusText(canvas, lines):
    """Draw status text at bottom of canvas."""
    for i, text in enumerate(reversed(lines)):
        y = canvas.shape[0] - 20 - (i * 25)
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

def createCanvasAndState(frame):
    """Optimized main vision processing pipeline."""
    global stableScreenshot, isStable
    
    # Return cached stable state if available
    if isStable and stableScreenshot is not None:
        return stableScreenshot, {}
    
    canvas = frame.copy()
    
    # Single ArUco detection call
    ids, centers, cornersMap, _ = detectAruco(frame)
    zone = buildOperatingZone(centers)
    pixelsPerMm = calculateScale(cornersMap)
    
    # Early exit if no origin marker
    if ORIGIN_ID not in centers:
        return canvas, {}
    
    originPx = centers[ORIGIN_ID]
    
    # Obstacle detection and processing
    obstacles = []
    obstacleVerticesData = []
    if zone.get('isComplete'):
        obstacles = detectObstacles(frame, zone)
        if obstacles:
            obstacleVerticesData, _ = processObstacles(obstacles, pixelsPerMm, originPx, printInfo=False)
            canvas = drawObstacles(canvas, obstacles, pixelsPerMm, originPx)
    
    # Zone visualization
    if zone.get('isComplete'):
        canvas = drawOperatingZone(canvas, zone)
    
    # Calculate world coordinates efficiently
    zoneCornersMm = goalZone = robotZone = robotThetaZone = None
    
    if zone.get('isComplete'):
        corners_px = zone['corners']
        zoneCornersMm = [
            ((corner_px[0] - originPx[0]) / pixelsPerMm, (originPx[1] - corner_px[1]) / pixelsPerMm)
            for corner_px in corners_px
        ]
    
    # Draw coordinate axes
    _drawCoordinateAxes(canvas, originPx)
    
    # Calculate goal and robot positions
    if GOAL_ID in centers:
        goalPx = centers[GOAL_ID]
        goalZone = ((goalPx[0] - originPx[0]) / pixelsPerMm, (originPx[1] - goalPx[1]) / pixelsPerMm)
    
    rCx, rCy, robotTheta = robotWorldPose(centers, cornersMap, ROBOT_ID)
    if rCx is not None and rCy is not None:
        robotZone = ((float(rCx) - originPx[0]) / pixelsPerMm, (originPx[1] - float(rCy)) / pixelsPerMm)
        if robotTheta is not None:
            robotThetaZone = np.degrees(robotTheta)
            if robotThetaZone < 0:
                robotThetaZone += 360
    
    # Draw markers
    _drawMarkers(canvas, centers, cornersMap)
    
    # Build state
    state = {
        'zoneCorners': zoneCornersMm,
        'goal': goalZone,
        'robot': robotZone,
        'robotTheta': robotThetaZone,
        'obstacles': [obs.toDict() for obs in obstacles],
        'obstacleVertices': obstacleVerticesData
    }
    
    # Handle stability and UI
    canvas = _handleStabilityAndUI(canvas, state, centers, zoneCornersMm, robotZone, robotThetaZone, goalZone)
    
    return canvas, state

def getOperatingState(state):
    """Convert internal state to structured output format."""
    output = {
        'zoneCorners': state.get('zoneCorners'),
        'obstacles': state.get('obstacles', []),
        'obstacleVertices': state.get('obstacleVertices', []),
        'goal': None,
        'robot': None,
        'timestamp': cv2.getTickCount() / cv2.getTickFrequency()
    }
    
    if state.get('goal'):
        output['goal'] = {'x': state['goal'][0], 'y': state['goal'][1]}
    
    if state.get('robot') and state.get('robotTheta') is not None:
        output['robot'] = {
            'x': state['robot'][0], 
            'y': state['robot'][1], 
            'theta': state['robotTheta']
        }
    
    return output