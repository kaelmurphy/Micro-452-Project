import cv2
import numpy as np
from camera_setup import CameraStream

# Global variables for temporal smoothing
prev_contours = []
contour_history = []
HISTORY_SIZE = 5

class Obstacle:
    def __init__(self, contour, area):
        self.contour = contour
        self.area = area
    
    def toDict(self):
        return {
            'contour': self.contour.tolist(),
            'area': self.area
        }

def detectObstacles(frame, zone):
    """Detect solid filled obstacles within the operating zone, ignoring ArUco markers."""
    global prev_contours, contour_history
    
    # Extract zone corners for masking
    operatingZone = zone.get('corners') if zone else None
    
    # Get ArUco marker positions to exclude them from obstacle detection
    from aruco_utils import detectAruco
    ids, centers, cornersMap, annotated = detectAruco(frame)
    
    # Create exclusion zones around ALL ArUco markers
    exclusion_zones = []
    robotId = 8
    goalId = 9
    cornerIds = [0, 1, 2, 3]
    
    # Exclude all detected ArUco markers
    for marker_id in ids if ids is not None else []:
        if marker_id in centers:
            marker_center = centers[marker_id]
            exclusion_zones.append((marker_center, 150))  # Large exclusion radius
    
    # Convert to HSV for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Apply zone masking if operatingZone is provided
    if operatingZone is not None:
        # Create mask for the operating zone
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(operatingZone, dtype=np.int32)], (255,))
        hsv = cv2.bitwise_and(hsv, hsv, mask=mask)
    
    # Create exclusion mask for ALL ArUco markers
    exclusion_mask = np.ones(hsv.shape[:2], dtype=np.uint8) * 255
    for center, radius in exclusion_zones:
        cv2.circle(exclusion_mask, (int(center[0]), int(center[1])), radius, (0,), -1)
    
    # Apply exclusion mask
    hsv = cv2.bitwise_and(hsv, hsv, mask=exclusion_mask)
    
    # Define color range for obstacles - wider range for better detection
    # Much wider HSV range for green obstacles with generous tolerance
    lower_green = np.array([50, 20, 20])    # Wider lower bound
    upper_green = np.array([100, 255, 255]) # Wider upper bound
    binary = cv2.inRange(hsv, lower_green, upper_green)
    
    # Alternative colors (commented out):
    # For RED obstacles:
    # lower_red1 = np.array([0, 50, 50])
    # upper_red1 = np.array([10, 255, 255])
    # lower_red2 = np.array([170, 50, 50])
    # upper_red2 = np.array([180, 255, 255])
    # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    # binary = cv2.bitwise_or(mask1, mask2)
    
    # For BLUE obstacles:
    # lower_blue = np.array([100, 50, 50])
    # upper_blue = np.array([130, 255, 255])
    # binary = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Morphological operations to clean up and fill holes
    kernel = np.ones((5,5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # Find contours from the binary image
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    current_contours = []
    
    # Collect all valid contours
    for i, contour in enumerate(contours):
        if len(contour) < 3:  # Skip contours with too few points
            continue
        
        # Calculate contour area and filter by size range
        area = cv2.contourArea(contour)
        min_area = 500    # Reduced minimum area for better detection
        max_area = 50000  # Maximum area to avoid large background regions
        if area < min_area or area > max_area:
            continue
            
        # Approximate contour with fewer vertices (Douglas-Peucker algorithm)
        epsilon = 0.005 * cv2.arcLength(contour, True)  # Much smaller epsilon for more corner detection
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Collect valid polygons - allow any number of corners
        if len(approx) >= 3:  # Remove upper limit to detect all corners
            current_contours.append(approx)
    
    # Simple temporal smoothing - average with previous frame
    contour_history.append(current_contours)
    if len(contour_history) > 2:  # Keep only 2 frames
        contour_history.pop(0)
    
    # Draw contours (use current frame if no history, otherwise blend)
    if len(contour_history) >= 2:
        # Only draw contours that appear in both recent frames
        display_contours = []
        for current in current_contours:
            current_area = cv2.contourArea(current)
            # Check if similar contour exists in previous frame
            for prev_frame in contour_history[:-1]:
                for prev in prev_frame:
                    prev_area = cv2.contourArea(prev)
                    if abs(current_area - prev_area) < current_area * 0.4:  # 40% tolerance
                        display_contours.append(current)
                        break
    else:
        display_contours = current_contours
    
    # Convert to Obstacle objects
    obstacles = []
    for contour in display_contours:
        area = cv2.contourArea(contour)
        obstacles.append(Obstacle(contour, area))
    
    return obstacles

def drawObstacles(canvas, obstacles, pixelsPerMm, origin):
    """Draw obstacles on the canvas in red."""
    for obstacle in obstacles:
        contour = np.array(obstacle.contour, dtype=np.int32)
        
        # Fill the enclosed area with red shading
        cv2.fillPoly(canvas, [contour], (0, 0, 200))  # Red fill
        
        # Draw the polygon outline in dark red
        cv2.polylines(canvas, [contour], True, (0, 0, 100), 2)  # Dark red outline
        
        # Optional: Draw corner points as small circles
        for point in contour:
            x, y = point[0]
            cv2.circle(canvas, (x, y), 3, (255, 0, 0), -1)  # Blue dots for corners
    
    return canvas

def camera2(frame, operatingZone):
    """Legacy function for standalone operation."""
    obstacles = detectObstacles(frame, {'corners': operatingZone})
    
    # Create white canvas for standalone display
    canvas = np.ones_like(frame) * 255
    
    # Draw obstacles in red
    for obstacle in obstacles:
        contour = np.array(obstacle.contour, dtype=np.int32)
        
        # Fill the enclosed area with red shading
        cv2.fillPoly(canvas, [contour], (0, 0, 200))
        
        # Draw the polygon outline in dark red
        cv2.polylines(canvas, [contour], True, (0, 0, 100), 2)
        
        # Optional: Draw corner points as small circles
        for point in contour:
            x, y = point[0]
            cv2.circle(canvas, (x, y), 3, (255, 0, 0), -1)
    
    return canvas