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
            rawVertices = [(point[0][0], point[0][1]) for point in self.contour]
            self._vertices, _ = _ensureCounterClockwise(rawVertices)
        return self._vertices
    
    def getVerticesWorldCoords(self, pixelsPerMm, origin):
        """
        Convert pixel coordinates to world coordinates with caching.
        """
        # Check if we can use cached result
        if (self._worldVertices is not None and 
            self._lastPixelsPerMm == pixelsPerMm and 
            self._lastOrigin == origin):
            return self._worldVertices
        
        # Calculate and cache new world coordinates
        self._worldVertices = [
            ((xPixel - origin[0]) / pixelsPerMm, (origin[1] - yPixel) / pixelsPerMm)
            for xPixel, yPixel in self.vertices
        ]
        self._lastPixelsPerMm = pixelsPerMm
        self._lastOrigin = origin
        return self._worldVertices
    
    def toDict(self):
        return {
            'contour': self.contour.tolist(),
            'area': self.area,
            'vertices': self.vertices
        }

def detectObstacles(frame, zone, minArea=500, maxArea=50000, colorRange=None):
    """
    Detect colored obstacles with optimized processing pipeline.
    """
    global contourHistory
    
    # Early exit if zone is incomplete
    if not zone.get('isComplete'):
        return []
    
    # Use provided color range or default green
    if colorRange is None:
        lowerGreen = np.array([50, 20, 20])
        upperGreen = np.array([100, 255, 255])
    else:
        lowerGreen, upperGreen = colorRange
    
    # Optimized HSV conversion and masking
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    binary = cv2.inRange(hsv, lowerGreen, upperGreen)
    
    # Single morphological operation with larger kernel for efficiency
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    # Find contours with optimized parameters
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    
    # Vectorized area calculation and filtering
    validContours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if minArea <= area <= maxArea and len(contour) >= 3:
            # Optimized polygon approximation
            epsilon = 0.01 * cv2.arcLength(contour, True)  # Slightly larger epsilon
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) >= 3:
                validContours.append(approx)
    
    # Temporal smoothing with improved similarity check
    contourHistory.append(validContours)
    if len(contourHistory) > HISTORY_SIZE:
        contourHistory.pop(0)
    
    # Stability filtering - find contours that appear consistently
    if len(contourHistory) >= 2:
        stableContours = []
        areaTolerance = 0.3
        
        for current in validContours:
            currentArea = cv2.contourArea(current)
            if currentArea == 0:
                continue
                
            # Check if similar contour exists in previous frames
            isStable = False
            for prevFrame in contourHistory[:-1]:
                for prev in prevFrame:
                    prevArea = cv2.contourArea(prev)
                    if abs(currentArea - prevArea) / currentArea < areaTolerance:
                        isStable = True
                        break
                if isStable:
                    break
            
            if isStable:
                stableContours.append(current)
    else:
        stableContours = validContours
    
    # Create obstacle objects
    return [Obstacle(contour, cv2.contourArea(contour)) for contour in stableContours]

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
    
    # Process all obstacles
    for i, obstacle in enumerate(obstacles):
        worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
        orderedVertices, wasAlreadyCCW = _ensureCounterClockwise(worldVertices)
        centroid = _calculateCentroid(orderedVertices)
        
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
    
    for i, obstacle in enumerate(obstacles):
        contour = np.array(obstacle.contour, dtype=np.int32)
        
        # Efficient polygon rendering
        cv2.fillPoly(canvas, [contour], (0, 0, 200))
        cv2.polylines(canvas, [contour], True, (0, 0, 100), 2)
        
        # Only show detailed info if requested
        if showCoordinates or showVertexNumbers:
            worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
            
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
        
        # Obstacle ID at centroid
        if len(contour) > 0:
            centerX = int(np.mean(contour[:, 0, 0]))
            centerY = int(np.mean(contour[:, 0, 1]))
            cv2.putText(canvas, f"Obs{i+1}", (centerX-12, centerY), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
    
    return canvas