import cv2
import numpy as np
from camera_setup import CameraStream
from feed_processing import createCanvasAndState, resetStability

def main():
    cam = CameraStream(index=0, width=1920, height=1080, fps=30).start()
    show_clean_view = True
    window_name = "Vision System"
    frame_skip = 0  # Skip processing every nth frame for speed
    displayFrame = None  # Initialize display frame
    
    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue
            
            # Skip heavy processing on some frames for speed
            frame_skip = (frame_skip + 1) % 2
            
            if show_clean_view:
                # Create clean blank canvas with only essential elements
                canvas, state = createCanvasAndState(frame)
                clean_canvas = np.ones_like(frame) * 255
                canvas, state = createCleanCanvas(frame, clean_canvas)
                title = "Clean View - 'q' for live feed | 'r' to reset | 'x' to quit"
            else:
                # Show live feed with overlay
                canvas, state = createCanvasAndState(frame)
                title = "Live Feed - 'q' for clean view | 'r' to reset | 'x' to quit"
            
            # Optimized display processing
            if frame_skip == 0 or displayFrame is None:  # Resize when needed
                displayFrame = cv2.resize(canvas, (960, 540), interpolation=cv2.INTER_LINEAR)  # Fixed size, faster
            cv2.imshow(window_name, displayFrame)
            cv2.setWindowTitle(window_name, title)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                show_clean_view = not show_clean_view
            elif key == ord('r'):
                resetStability()
            elif key == ord('x') or key == 27:  # 'x' key or ESC to exit
                break
                
    finally:
        cam.stop()
        cv2.destroyAllWindows()

def createCleanCanvas(frame, clean_canvas):
    """Create a clean canvas with only operating zone, robot, goal, and obstacles."""
    from aruco_utils import detectAruco, buildOperatingZone
    from draw_utils import drawOperatingZone
    from coord_utils import robotWorldPose
    from obstacle import detectObstacles, drawObstacles
    from feed_processing import calculateScale, ROBOT_ID, GOAL_ID, ORIGIN_ID
    
    robotId = ROBOT_ID
    goalId = GOAL_ID
    
    # Detect ArUco markers
    ids, centers, cornersMap, annotated = detectAruco(frame)
    zone = buildOperatingZone(centers)
    
    # Draw operating zone
    if zone.get('isComplete'):
        pts = np.array(zone['corners'], dtype=np.int32)
        overlay = clean_canvas.copy()
        cv2.fillPoly(overlay, [pts], (0, 255, 255))  # Yellow fill
        cv2.addWeighted(overlay, 0.15, clean_canvas, 0.85, 0, clean_canvas)
        cv2.polylines(clean_canvas, [pts], True, (0, 255, 255), 3)  # Yellow border
        
        # Detect and draw obstacles
        if ORIGIN_ID in centers:
            obstacles = detectObstacles(frame, zone)
            if obstacles:
                pixelsPerMm = calculateScale(cornersMap)
                origin = centers[ORIGIN_ID]
                clean_canvas = drawObstacles(clean_canvas, obstacles, pixelsPerMm, origin, 
                                           showCoordinates=False, showVertexNumbers=False)
    
    # Draw robot
    if robotId in centers:
        robot_center = (int(centers[robotId][0]), int(centers[robotId][1]))
        cv2.circle(clean_canvas, robot_center, 15, (255, 0, 0), -1)  # Blue circle
        cv2.putText(clean_canvas, "ROBOT", (robot_center[0] + 20, robot_center[1] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # Draw robot orientation arrow
        if robotId in cornersMap:
            corners = cornersMap[robotId]
            top_mid = ((corners[0] + corners[1]) / 2).astype(int)
            cv2.arrowedLine(clean_canvas, robot_center, tuple(top_mid), (255, 150, 150), 3)
    
    # Draw goal
    if goalId in centers:
        goal_center = (int(centers[goalId][0]), int(centers[goalId][1]))
        cv2.circle(clean_canvas, goal_center, 15, (0, 255, 0), -1)  # Green circle
        cv2.putText(clean_canvas, "GOAL", (goal_center[0] + 20, goal_center[1] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Draw coordinate axes if origin is detected
    if ORIGIN_ID in centers:
        originPx = centers[ORIGIN_ID]
        axisLength = 100
        xEnd = (int(originPx[0] + axisLength), int(originPx[1]))
        yEnd = (int(originPx[0]), int(originPx[1] - axisLength))
        cv2.arrowedLine(clean_canvas, (int(originPx[0]), int(originPx[1])), xEnd, (0, 0, 255), 3)
        cv2.arrowedLine(clean_canvas, (int(originPx[0]), int(originPx[1])), yEnd, (0, 255, 0), 3)
        cv2.putText(clean_canvas, "X", (xEnd[0] + 5, xEnd[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(clean_canvas, "Y", (yEnd[0] - 15, yEnd[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.circle(clean_canvas, (int(originPx[0]), int(originPx[1])), 8, (0, 0, 0), -1)  # Black origin point
    
    return clean_canvas, {}

if __name__ == "__main__":
    main()