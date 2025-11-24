import cv2
import numpy as np

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