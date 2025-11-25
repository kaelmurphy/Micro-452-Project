import cv2
import numpy as np
import math
from .camera_setup import CameraStream
from .feed_processing import convertPixelsToWorldCoords

# Global variables for temporal smoothing
contourHistory = []
HISTORY_SIZE = 3  # Increased for better tracking consistency

class Obstacle:
    def __init__(self, contour, area):
        self.contour = contour
        self.area = area
        self._vertices = self._worldVertices = None
    
    @property
    def vertices(self):
        if self._vertices is None:
            # Use approxPolyDP with higher epsilon for fewer, more prominent vertices
            peri = cv2.arcLength(self.contour, True)
            approx = cv2.approxPolyDP(self.contour, 0.02 * peri, True)
            # Round coordinates for stability
            self._vertices = [(int(round(p[0][0])), int(round(p[0][1]))) for p in approx]
        return self._vertices
    
    def getVerticesWorldCoords(self, cornersMap, origin):
        world_vertices = convertPixelsToWorldCoords(self.vertices, origin, cornersMap)
        return self._sortVerticesCCWFromBottomLeft(world_vertices)
    
    def _sortVerticesCCWFromBottomLeft(self, vertices):
        if len(vertices) < 3:
            return vertices
        
        cx = sum(v[0] for v in vertices) / len(vertices)
        cy = sum(v[1] for v in vertices) / len(vertices)
        
        def angle(vertex):
            angle = math.atan2(vertex[1] - cy, vertex[0] - cx)
            return angle + 2 * math.pi if angle < 0 else angle
        
        sorted_vertices = sorted(vertices, key=angle)
        bottom_left_idx = min(range(len(sorted_vertices)), 
                             key=lambda i: (sorted_vertices[i][1], sorted_vertices[i][0]))
        return sorted_vertices[bottom_left_idx:] + sorted_vertices[:bottom_left_idx]
    
    def toDict(self):
        return {'contour': self.contour.tolist(), 'area': self.area, 'vertices': self.vertices}

def detectObstacles(frame, zone, minArea=4000, maxArea=50000, colorRange=None):
    """
    Detect colored obstacles with optimized processing pipeline.
    """
    # Early exit if zone is incomplete
    if not zone.get('isComplete'):
        return []

    # Create zone mask to restrict detection to operating area only
    height, width = frame.shape[:2]
    zone_mask = np.zeros((height, width), dtype=np.uint8)
    zone_corners = zone.get('corners', [])
    if len(zone_corners) >= 3:
        zone_points = np.array(zone_corners, dtype=np.int32)
        cv2.fillPoly(zone_mask, [zone_points], 255)
    else:
        zone_mask[:] = 255

    # Use BGR filtering for new obstacle color (#3d7c80, BGR: 128, 124, 61)
    # BGR: (128, 124, 61) -- allow a range around this value
    lower_obstacle = np.array([80, 70, 20])     # BGR: tighter lower bound
    upper_obstacle = np.array([180, 170, 110])  # BGR: tighter upper bound
    mask = cv2.inRange(frame, lower_obstacle, upper_obstacle)

    # Apply zone mask
    mask = cv2.bitwise_and(mask, zone_mask)

    # Morphology for clean contours
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if minArea <= area <= maxArea and len(contour) >= 3:
            obstacles.append(Obstacle(contour, area))
    return obstacles


def processObstacles(obstacles, pixelsPerMm, origin, printInfo=False, verbose=False):
    """Process obstacles to extract vertices in world coordinates."""
    if not obstacles:
        return [], []
    
    obstacleData, allVertices = [], []
    
    for i, obstacle in enumerate(obstacles):
        worldVertices = obstacle.getVerticesWorldCoords(pixelsPerMm, origin)
        if worldVertices:
            centroid = (sum(v[0] for v in worldVertices) / len(worldVertices),
                       sum(v[1] for v in worldVertices) / len(worldVertices))
        else:
            centroid = (0.0, 0.0)
        
        obstacleData.append({
            'id': i + 1,
            'vertices': worldVertices,
            'area': obstacle.area,
            'vertexCount': len(worldVertices),
            'centroid': centroid,
            'isCounterClockwise': _isCounterClockwise(worldVertices)
        })
        allVertices.extend(worldVertices)
    
    return obstacleData, allVertices

def _isCounterClockwise(vertices):
    """Check if vertices are in counter-clockwise order."""
    if len(vertices) < 3:
        return True
    
    signed_area = sum((vertices[(i + 1) % len(vertices)][0] - vertices[i][0]) * 
                     (vertices[(i + 1) % len(vertices)][1] + vertices[i][1]) 
                     for i in range(len(vertices)))
    return signed_area < 0

# Utility functions for polygon processing have been moved to class methods

def drawObstacles(canvas, obstacles, cornersMap, origin, showCoordinates=False, showVertexNumbers=False):
    """Minimal obstacle drawing."""
    if not obstacles:
        return canvas
    
    for obstacle in obstacles:
        contour = np.array(obstacle.contour, dtype=np.int32)
        cv2.fillPoly(canvas, [contour], (0, 0, 255))  # Solid red
        cv2.polylines(canvas, [contour], True, (0, 0, 255), 1)  # Red outline
    
    return canvas