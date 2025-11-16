import cv2
import numpy as np

class Obstacle:
    def __init__(self, obstacleId, vertices=None):
        self.id = obstacleId
        self.vertices = vertices if vertices else []
    
    def addVertex(self, x, y):
        self.vertices.append((float(x), float(y)))
    
    def setVertices(self, vertices):
        self.vertices = [(float(x), float(y)) for x, y in vertices]
    
    def getVertices(self):
        return self.vertices
    
    def toDict(self):
        return {'id': self.id, 'vertices': self.vertices}
    
    def __repr__(self):
        return f"Obstacle(id={self.id}, vertices={len(self.vertices)})"

def detectObstacles(frame, zone, pixelsPerMm, bl_px, centers, cornersMap):
    """detect obstacles using edge detection and contour analysis."""
    if not zone.get('isComplete'):
        return []
    
    # create mask for zone area
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    zone_pts = np.array(zone['corners'], dtype=np.int32)
    cv2.fillPoly(mask, [zone_pts], (255,))
    
    # create exclusion mask for aruco markers
    aruco_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for marker_id, corners in cornersMap.items():
        if corners is not None and len(corners) == 4:
            expanded_corners = []
            center = np.mean(corners, axis=0)
            for corner in corners:
                direction = corner - center
                expanded_corner = center + direction * 1.5
                expanded_corners.append(expanded_corner.astype(int))
            
            pts = np.array(expanded_corners, dtype=np.int32)
            cv2.fillPoly(aruco_mask, [pts], (255,))
    
    # subtract aruco areas from zone mask
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(aruco_mask))
    
    # convert to grayscale and apply blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    
    # edge detection with higher thresholds
    edges = cv2.Canny(blurred, 80, 200)
    
    # morphological operations to clean edges
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
    
    # apply zone mask to edges
    edges_masked = cv2.bitwise_and(edges, mask)
    
    # find contours
    contours, _ = cv2.findContours(edges_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    obstacles = []
    obstacle_id = 0
    
    for contour in contours:
        # filter small contours
        area = cv2.contourArea(contour)
        if area < 1500:
            continue
            
        # filter elongated contours
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        if aspect_ratio > 5 or aspect_ratio < 0.2:
            continue
        
        # approximate contour to polygon
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # reasonable vertex count
        if len(approx) < 3 or len(approx) > 20:
            continue
        
        # convert vertices to mm coordinates
        vertices_mm = []
        for vertex in approx:
            px_x, px_y = vertex[0]
            mm_x = (px_x - bl_px[0]) / pixelsPerMm
            mm_y = (bl_px[1] - px_y) / pixelsPerMm
            vertices_mm.append((mm_x, mm_y))
        
        obstacle = Obstacle(obstacle_id, vertices_mm)
        obstacles.append(obstacle)
        obstacle_id += 1
    
    return obstacles

def drawObstacles(canvas, obstacles, pixelsPerMm, bl_px):
    """draw obstacles on canvas."""
    for obstacle in obstacles:
        if not obstacle.vertices:
            continue
            
        # convert mm coordinates back to pixels
        pixel_vertices = []
        for mm_x, mm_y in obstacle.vertices:
            px_x = int(mm_x * pixelsPerMm + bl_px[0])
            px_y = int(bl_px[1] - mm_y * pixelsPerMm)
            pixel_vertices.append([px_x, px_y])
        
        if len(pixel_vertices) < 3:
            continue
            
        pts = np.array(pixel_vertices, dtype=np.int32)
        
        # draw filled obstacle with transparency
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [pts], (0, 0, 255))
        cv2.addWeighted(overlay, 0.3, canvas, 0.7, 0, canvas)
        
        # draw obstacle outline
        cv2.polylines(canvas, [pts], True, (0, 0, 255), 2)
        
        # draw vertices
        for vertex in pts:
            cv2.circle(canvas, tuple(vertex), 3, (0, 0, 255), -1)
        
        # label obstacle
        centroid = np.mean(pts, axis=0).astype(int)
        cv2.putText(canvas, f"OBS{obstacle.id}", (centroid[0] - 20, centroid[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    return canvas