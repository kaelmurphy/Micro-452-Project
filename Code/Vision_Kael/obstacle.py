import cv2
import numpy as np
from camera_setup import CameraStream

# Global variables for temporal smoothing
contourHistory = []
HISTORY_SIZE = 2  # Keep only 2 frames for noise reduction

class Obstacle:
    def __init__(self, contour, area):
        self.contour = contour
        self.area = area
        self._vertices = None  # Lazy loading
        self._worldVertices = None  # Cache world coordinates
        self._lastPixelsPerMm = None
        self._lastOrigin = None
    
    @property
    def vertices(self):
        """
        Lazy load vertices from contour with counter-clockwise ordering.
        """
        if self._vertices is None:
            # Handle different contour formats from OpenCV
            if len(self.contour.shape) == 3:  # Shape: (n_points, 1, 2)
                rawVertices = [(int(point[0][0]), int(point[0][1])) for point in self.contour]
            else:  # Shape: (n_points, 2)
                rawVertices = [(int(point[0]), int(point[1])) for point in self.contour]
            self._vertices, _ = _ensureCounterClockwise(rawVertices)
        return self._vertices
    
    def getVerticesWorldCoords(self, pixelsPerMm, origin):
        """
        Convert pixel coordinates to world coordinates with caching.
        World coordinate system: origin at ArUco marker ID 3, X+ right, Y+ up
        """
        # Check if we can use cached result
        if (self._worldVertices is not None and 
            self._lastPixelsPerMm == pixelsPerMm and 
            self._lastOrigin == origin):
            return self._worldVertices
        
        # Calculate and cache new world coordinates
        # World coordinate system: origin at ArUco marker ID 3
        # X+ rightward (positive X = right of origin)
        # Y+ upward (positive Y = above origin)
        self._worldVertices = []
        for i, (xPixel, yPixel) in enumerate(self.vertices):
            # SAME FORMULA AS ROBOT AND GOAL:
            # X: rightward from origin (right = positive)
            worldX = (float(xPixel) - float(origin[0])) / pixelsPerMm
            # Y: upward from origin (up = positive, flip pixel Y)
            worldY = (float(origin[1]) - float(yPixel)) / pixelsPerMm
            self._worldVertices.append((worldX, worldY))
            
            # Debug coordinate conversion - compare with robot/goal formula
            if len(self._worldVertices) <= 2:
                direction_x = "RIGHT" if xPixel > origin[0] else "LEFT"
                direction_y = "ABOVE" if yPixel < origin[1] else "BELOW"
                print(f"OBSTACLE V{i}: Px({xPixel:.1f},{yPixel:.1f}) [{direction_x},{direction_y}] -> World({worldX:.2f},{worldY:.2f})")
                print(f"    Formula: X = ({xPixel:.1f} - {origin[0]:.1f}) / {pixelsPerMm:.3f} = {worldX:.2f}")
                print(f"    Formula: Y = ({origin[1]:.1f} - {yPixel:.1f}) / {pixelsPerMm:.3f} = {worldY:.2f}")
        
        self._lastPixelsPerMm = pixelsPerMm
        self._lastOrigin = origin
        return self._worldVertices
    
    def toDict(self):
        return {
            'contour': self.contour.tolist(),
            'area': self.area,
            'vertices': self.vertices
        }

def detectObstacles(frame, zone, minArea=300, maxArea=50000, colorRange=None):
    """
    Detect colored obstacles with optimized processing pipeline.
    """
    global contourHistory
    
    # Early exit if zone is incomplete
    if not zone.get('isComplete'):
        return []
    
    # Use provided color range or default RGB-based range
    if colorRange is None:
        # RGB range (30,75,75) to (45,105,105) - widened slightly
        # Convert to BGR for OpenCV (Blue, Green, Red)
        lowerTeal = np.array([70, 70, 25])    # BGR lower bound (widened by 5)
        upperTeal = np.array([110, 110, 50])  # BGR upper bound (widened by 5)
        use_rgb = True
    else:
        lowerTeal, upperTeal = colorRange
        use_rgb = False
    
    # Use RGB color space instead of HSV for direct RGB filtering
    if use_rgb:
        # Work directly in RGB/BGR color space
        color_frame = frame  # Already in BGR format
        binary = cv2.inRange(color_frame, lowerTeal, upperTeal)
    else:
        # Fallback to HSV if custom range provided
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, lowerTeal, upperTeal)
    
    # Pre-create kernel for morphology (reuse)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    # Apply opening to remove noise, then closing to fill gaps
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    # Find contours with optimized parameters
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    
    # Vectorized filtering with early exits
    validContours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if minArea <= area <= maxArea:
            contour_len = len(contour)
            if contour_len >= 3:
                # Optimized polygon approximation with early area check
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) >= 3:
                    validContours.append(approx)
    
    # Optimized temporal smoothing
    contourHistory.append(validContours)
    if len(contourHistory) > HISTORY_SIZE:
        contourHistory.pop(0)
    
    # Fast stability filtering with balanced tolerance
    if len(contourHistory) >= 2:
        stableContours = []
        areaTolerance = 0.35  # Balanced tolerance
        
        # Pre-compute areas for current frame
        currentAreas = [cv2.contourArea(c) for c in validContours]
        
        for idx, current in enumerate(validContours):
            currentArea = currentAreas[idx]
            if currentArea == 0:
                continue
            
            # Fast similarity check against previous frames
            isStable = any(
                abs(currentArea - cv2.contourArea(prev)) / currentArea < areaTolerance
                for prevFrame in contourHistory[:-1]
                for prev in prevFrame
                if cv2.contourArea(prev) > 0
            )
            
            if isStable:
                stableContours.append(current)
    else:
        stableContours = validContours
    
    # Create obstacle objects with pre-computed areas
    obstacles_found = [Obstacle(contour, cv2.contourArea(contour)) for contour in stableContours]
    
    # Debug: Print detection parameters when obstacles are found
    if len(obstacles_found) > 0:
        if use_rgb:
            print(f"\\nRGB DETECTION: Range [{lowerTeal[2]},{lowerTeal[1]},{lowerTeal[0]}] to [{upperTeal[2]},{upperTeal[1]},{upperTeal[0]}]")
        else:
            print(f"\\nHSV DETECTION: Range [{lowerTeal[0]}-{upperTeal[0]}, {lowerTeal[1]}-{upperTeal[1]}, {lowerTeal[2]}-{upperTeal[2]}]")
        print(f"Found {len(obstacles_found)} obstacles (min area: {minArea} px)")
        print("Using direct RGB color filtering")
    
    return obstacles_found

def processObstacles(obstacles, pixelsPerMm, origin, printInfo=False, verbose=False):
    """
    Process obstacles to extract vertices and optionally print information.
    """
    if not obstacles:
        if printInfo:
            print("No obstacles detected")
        return [], []
    
    obstacleData = []
    allVertices = []
    
    # Pre-compute origin coordinates for batch processing
    origin_x, origin_y = origin
    
    # Process all obstacles with optimized coordinate conversion
    for i, obstacle in enumerate(obstacles):
        worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
        orderedVertices, wasAlreadyCCW = _ensureCounterClockwise(worldVertices)
        
        # Fast centroid calculation
        if orderedVertices:
            centroid_x = sum(v[0] for v in orderedVertices) / len(orderedVertices)
            centroid_y = sum(v[1] for v in orderedVertices) / len(orderedVertices)
            centroid = (centroid_x, centroid_y)
        else:
            centroid = (0.0, 0.0)
        
        obstacleInfo = {
            'id': i + 1,
            'vertices': orderedVertices,
            'area': obstacle.area,
            'vertexCount': len(orderedVertices),
            'centroid': centroid,
            'isCounterClockwise': wasAlreadyCCW
        }
        obstacleData.append(obstacleInfo)
        allVertices.extend(orderedVertices)
    
    # Optional printing
    if printInfo:
        print(f"\n=== {len(obstacles)} OBSTACLES DETECTED ===")
        for obs in obstacleData:
            centroid = obs['centroid']
            ordering = "CCW" if obs['isCounterClockwise'] else "CW"
            print(f"Obstacle {obs['id']}: Area={obs['area']:.0f}px² Vertices={obs['vertexCount']} Center=({centroid[0]:.1f},{centroid[1]:.1f})mm Order={ordering}")
            
            if verbose:
                for j, (x, y) in enumerate(obs['vertices']):
                    print(f"  V{j}: ({x:.2f}, {y:.2f})mm")
        
        print(f"Total vertices: {len(allVertices)} (all counter-clockwise)")
    
    return obstacleData, allVertices

# Utility functions for polygon processing
def _ensureCounterClockwise(vertices):
    """
    Ensure vertices are ordered counter-clockwise using shoelace formula.
    """
    if len(vertices) < 3:
        return vertices, True
    
    # Calculate signed area using shoelace formula
    signedArea = sum((vertices[(i+1)%len(vertices)][0] - vertices[i][0]) * 
                    (vertices[(i+1)%len(vertices)][1] + vertices[i][1]) 
                    for i in range(len(vertices)))
    
    # In our formula: negative = CCW, positive = CW
    # If positive, vertices are clockwise → reverse them
    return (vertices[::-1], False) if signedArea > 0 else (vertices, True)

def _calculateCentroid(vertices):
    """
    Calculate geometric centroid of polygon vertices.
    """
    if not vertices:
        return (0.0, 0.0)
    return (sum(v[0] for v in vertices) / len(vertices),
            sum(v[1] for v in vertices) / len(vertices))

def drawObstacles(canvas, obstacles, pixelsPerMm, origin, showCoordinates=True, showVertexNumbers=True):
    """
    Optimized obstacle drawing with configurable detail levels.
    """
    if not obstacles:
        return canvas
    
    # Pre-compute common values
    origin_x, origin_y = origin
    
    for i, obstacle in enumerate(obstacles):
        contour = np.array(obstacle.contour, dtype=np.int32)
        
        # Batch polygon rendering
        cv2.fillPoly(canvas, [contour], (0, 0, 200))
        cv2.polylines(canvas, [contour], True, (0, 0, 100), 2)
        
        # Batch centroid calculation for label
        if len(contour) > 0:
            center_x = int(np.mean(contour[:, 0, 0]))
            center_y = int(np.mean(contour[:, 0, 1]))
            cv2.putText(canvas, f"Obs{i+1}", (center_x-12, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
        
        # Only show detailed info if requested
        if showCoordinates or showVertexNumbers:
            worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
            
            # Batch process all vertex annotations
            for j, (point, worldCoord) in enumerate(zip(contour, worldVertices)):
                x, y = point[0]
                
                # Draw vertex point
                cv2.circle(canvas, (x, y), 4, (255, 0, 0), -1)
                
                if showCoordinates:
                    worldX, worldY = worldCoord
                    coordText = f"({worldX:.1f},{worldY:.1f})"
                    cv2.putText(canvas, coordText, (x + 8, y - 8), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
                
                if showVertexNumbers:
                    vertexText = f"V{j}"
                    cv2.putText(canvas, vertexText, (x - 12, y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)
    
    return canvas