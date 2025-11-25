import cv2
import numpy as np
from .aruco_utils import detectAruco, buildOperatingZone
from .draw_utils import drawOperatingZone
from .coord_utils import robotWorldPose, asXy
from .obstacle import detectObstacles, drawObstacles, processObstacles

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
    robot, goal, theta = state.get('robot'), state.get('goal'), state.get('robotTheta')
    if not (robot and goal and theta is not None):
        stabilityBuffer.clear()
        return False
    
    currentState = {'robot': robot, 'goal': goal, 'robotTheta': theta}
    
    # Check movement against previous frame
    if stabilityBuffer:
        last = stabilityBuffer[-1]
        if (np.linalg.norm(np.array(robot) - np.array(last['robot'])) > STABILITY_THRESHOLD or 
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
    
    # Draw markers
    from draw_utils import drawCoordinateSystem, drawRobotMarker, drawGoalMarker
    drawCoordinateSystem(canvas, originPx)
    
    if ROBOT_ID in centers:
        drawRobotMarker(canvas, centers[ROBOT_ID], cornersMap.get(ROBOT_ID))
    if GOAL_ID in centers:
        drawGoalMarker(canvas, centers[GOAL_ID])
    
    # Build state
    state = {
        'goal': goalZone,
        'robot': robotZone,
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