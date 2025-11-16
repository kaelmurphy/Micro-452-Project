import cv2
import numpy as np
from aruco_utils import detectAruco, buildOperatingZone
from draw_utils import drawOperatingZone
from coord_utils import robotWorldPose, asXy

robotId = 8
goalId = 9

# simple stability tracking
stability_buffer = []
stable_screenshot = None
is_stable = False
STABILITY_FRAMES = 100
STABILITY_THRESHOLD = 5.0

def resetStability():
    global is_stable, stable_screenshot, stability_buffer
    is_stable = False
    stable_screenshot = None
    stability_buffer.clear()

def calculateScale(cornersMap, marker_size_mm=100.0):
    """calculate pixels per mm using all available aruco marker edges.
    
    Args:
        cornersMap: Dictionary of marker_id -> corner points
        marker_size_mm: Physical size of markers in mm (default 100mm)
    
    Returns:
        pixels_per_mm: Scale factor for pixel to mm conversion
    """
    if not cornersMap:
        return 1.0
    
    edge_lengths = []
    
    # collect all edge lengths from all detected markers
    for marker_id, corners in cornersMap.items():
        if corners is not None and len(corners) == 4:
            for i in range(4):
                # calculate edge length in pixels
                p1 = corners[i]
                p2 = corners[(i + 1) % 4]
                edge_length_px = np.linalg.norm(p2 - p1)
                edge_lengths.append(edge_length_px)
    
    if not edge_lengths:
        return 1.0
    
    # use median instead of mean for better robustness against outliers
    avg_edge_length_px = np.median(edge_lengths)
    
    # calculate pixels per mm: known physical size / measured pixel size
    pixels_per_mm = avg_edge_length_px / marker_size_mm
    
    return pixels_per_mm

def pixelToWorld(pixel, pixelsPerMm, frameHeight):
    return (pixel[0] / pixelsPerMm, (frameHeight - pixel[1]) / pixelsPerMm)

def checkStability(state):
    global stability_buffer
    
    if not (state.get('robot') and state.get('goal') and state.get('robotTheta') is not None):
        stability_buffer.clear()
        return False
    
    current = {'robot': state['robot'], 'goal': state['goal'], 'robotTheta': state['robotTheta']}
    
    if stability_buffer:
        last = stability_buffer[-1]
        robot_diff = np.linalg.norm(np.array(current['robot']) - np.array(last['robot']))
        goal_diff = np.linalg.norm(np.array(current['goal']) - np.array(last['goal']))
        angle_diff = abs(current['robotTheta'] - last['robotTheta'])
        
        if robot_diff > STABILITY_THRESHOLD or goal_diff > STABILITY_THRESHOLD or angle_diff > 5.0:
            stability_buffer.clear()
    
    stability_buffer.append(current)
    return len(stability_buffer) == STABILITY_FRAMES

def createCanvasAndState(frame):
    global stable_screenshot, is_stable
    
    if is_stable and stable_screenshot is not None:
        return stable_screenshot, {}
    
    canvas = frame.copy()
    frameH, frameW = frame.shape[:2]
    
    ids, centers, cornersMap, annotated = detectAruco(frame)
    zone = buildOperatingZone(centers)
    pixelsPerMm = calculateScale(cornersMap)
    
    # only draw zone when all 4 corners found
    if zone.get('isComplete'):
        canvas = drawOperatingZone(canvas, zone)
    
    # calculate coordinates with bottom-left (ID 3) as origin (0,0)
    zoneCornersMm = None
    
    if 3 in centers and zone.get('isComplete'):
        # zone corners relative to bottom-left (ID 3) which becomes (0,0)
        bl_px = centers[3]  # bottom-left marker position in pixels
        corners_px = zone['corners']  # [bl, br, tr, tl] in pixel coordinates
        
        zoneCornersMm = []
        for corner_px in corners_px:
            # calculate relative position from bottom-left corner
            rel_x = (corner_px[0] - bl_px[0]) / pixelsPerMm
            rel_y = (bl_px[1] - corner_px[1]) / pixelsPerMm  # flip Y: bottom-left is (0,0)
            zoneCornersMm.append((rel_x, rel_y))
    
    # draw coordinate axes when origin found
    if 3 in centers:
        origin_px = centers[3]
        axis_length = 100
        
        x_end = (int(origin_px[0] + axis_length), int(origin_px[1]))
        y_end = (int(origin_px[0]), int(origin_px[1] - axis_length))
        
        cv2.arrowedLine(canvas, (int(origin_px[0]), int(origin_px[1])), x_end, (0, 0, 255), 3)
        cv2.arrowedLine(canvas, (int(origin_px[0]), int(origin_px[1])), y_end, (0, 255, 0), 3)
        cv2.putText(canvas, "X", (x_end[0] + 5, x_end[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(canvas, "Y", (y_end[0] - 15, y_end[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.circle(canvas, (int(origin_px[0]), int(origin_px[1])), 8, (255, 255, 255), -1)
    
    # calculate positions relative to bottom-left origin (0,0)
    goalZone = robotZone = robotThetaZone = None
    
    if goalId in centers and 3 in centers:
        bl_px = centers[3]
        goal_px = centers[goalId]
        goalZone = ((goal_px[0] - bl_px[0]) / pixelsPerMm, (bl_px[1] - goal_px[1]) / pixelsPerMm)
    
    rCx, rCy, robotTheta = robotWorldPose(centers, cornersMap, robotId)
    if rCx is not None and rCy is not None and 3 in centers:
        bl_px = centers[3]
        robotZone = ((float(rCx) - bl_px[0]) / pixelsPerMm, (bl_px[1] - float(rCy)) / pixelsPerMm)
        if robotTheta is not None:
            robotThetaZone = np.degrees(robotTheta)
            if robotThetaZone < 0:
                robotThetaZone += 360
    
    state = {
        'zoneCorners': zoneCornersMm,
        'goal': goalZone,
        'robot': robotZone,
        'robotTheta': robotThetaZone,
        'obstacles': []
    }
    
    # draw markers
    if robotId in centers:
        robot_center = (int(centers[robotId][0]), int(centers[robotId][1]))
        cv2.circle(canvas, robot_center, 8, (255, 0, 0), -1)
        cv2.putText(canvas, "ROBOT", (robot_center[0] + 15, robot_center[1] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # draw robot orientation line
        if robotId in cornersMap:
            corners = cornersMap[robotId]
            top_mid = ((corners[0] + corners[1]) / 2).astype(int)
            cv2.arrowedLine(canvas, robot_center, tuple(top_mid), (255, 150, 150), 3)
    
    if goalId in centers:
        goal_center = (int(centers[goalId][0]), int(centers[goalId][1]))
        cv2.circle(canvas, goal_center, 8, (0, 255, 0), -1)
        cv2.putText(canvas, "GOAL", (goal_center[0] + 15, goal_center[1] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # handle stability
    lines = []
    if not is_stable:
        if checkStability(state):
            is_stable = True
            # create enhanced screenshot with corner labels
            stable_screenshot = canvas.copy()
            
            # add corner coordinate labels to screenshot
            if 3 in centers and state['zoneCorners'] and len(state['zoneCorners']) == 4:
                bl, br, tr, tl = state['zoneCorners']
                # bottom-left (ID 3) is origin, others are relative to it
                corner_data = [(3, (0.0, 0.0), "BL"), (2, br, "BR"), (1, tr, "TR"), (0, tl, "TL")]
                
                for marker_id, coords, label in corner_data:
                    if marker_id in centers:
                        center = centers[marker_id]
                        text = f"{label} = ({coords[0]:.1f}, {coords[1]:.1f})"
                        cv2.putText(stable_screenshot, text, (int(center[0]) + 13, int(center[1]) + 25), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                        cv2.putText(stable_screenshot, text, (int(center[0]) + 13, int(center[1]) + 25), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # add robot coordinates and angle
            if robotZone and robotId in centers:
                center = centers[robotId]
                robot_text = f"ROBOT = ({robotZone[0]:.1f}, {robotZone[1]:.1f})"
                if robotThetaZone is not None:
                    robot_text += f", {robotThetaZone:.1f}"
                cv2.putText(stable_screenshot, robot_text, (int(center[0]) + 13, int(center[1]) + 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                cv2.putText(stable_screenshot, robot_text, (int(center[0]) + 13, int(center[1]) + 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 200), 2)
            
            # add goal coordinates
            if goalZone and goalId in centers:
                center = centers[goalId]
                goal_text = f"GOAL = ({goalZone[0]:.1f}, {goalZone[1]:.1f})"
                cv2.putText(stable_screenshot, goal_text, (int(center[0]) + 13, int(center[1]) + 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                cv2.putText(stable_screenshot, goal_text, (int(center[0]) + 13, int(center[1]) + 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            lines.append("SYSTEM LOCKED - COORDINATES STABLE")
        else:
            progress = len(stability_buffer) / STABILITY_FRAMES * 100
            lines.append(f"CALIBRATING... {progress:.0f}% ({len(stability_buffer)}/{STABILITY_FRAMES} frames)")
    else:
        lines.append("SYSTEM LOCKED - COORDINATES STABLE")
        lines.append("Press 'R' to recalibrate")
    
    # add status
    robot_status = "FOUND" if robotId in centers else "NOT FOUND"
    goal_status = "FOUND" if goalId in centers else "NOT FOUND"
    
    lines.extend([
        f"Robot ID {robotId}: {robot_status}",
        f"Goal ID {goalId}: {goal_status}"
    ])
    
    if state['zoneCorners'] and len(state['zoneCorners']) == 4:
        bl, br, tr, tl = state['zoneCorners']
        lines.extend([
            f"Zone (mm) - BL:({bl[0]:.1f},{bl[1]:.1f}) BR:({br[0]:.1f},{br[1]:.1f})",
            f"Zone (mm) - TL:({tl[0]:.1f},{tl[1]:.1f}) TR:({tr[0]:.1f},{tr[1]:.1f})"
        ])
    
    if state['goal']:
        gx, gy = state['goal']
        lines.append(f"Goal: x={gx:.1f}mm y={gy:.1f}mm")
    
    if state['robot']:
        rx, ry = state['robot']
        theta = f"{state['robotTheta']:.1f}" if state['robotTheta'] is not None else "n/a"
        lines.append(f"Robot: x={rx:.1f}mm y={ry:.1f}mm angle={theta}")
    
    # draw text
    for i, txt in enumerate(reversed(lines)):
        y = canvas.shape[0] - 20 - (i * 25)
        cv2.putText(canvas, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(canvas, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return canvas, state

def getOperatingState(state):
    output = {'zoneCorners': state.get('zoneCorners'), 'obstacles': [], 'goal': None, 'robot': None}
    
    if state.get('goal'):
        output['goal'] = {'x': state['goal'][0], 'y': state['goal'][1]}
        
    if state.get('robot') and state.get('robotTheta') is not None:
        output['robot'] = {'x': state['robot'][0], 'y': state['robot'][1], 'theta': state['robotTheta']}
        
    return output