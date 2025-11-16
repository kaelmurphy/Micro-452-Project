import cv2
import numpy as np

# Global storage for temporal filtering
obstacle_history = []
HISTORY_SIZE = 5  # balanced for stability
STABILITY_THRESHOLD = 3  # must appear in 3 out of 5 frames

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
    global obstacle_history
    
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
                expanded_corner = center + direction * 1.2
                expanded_corners.append(expanded_corner.astype(int))
            
            pts = np.array(expanded_corners, dtype=np.int32)
            cv2.fillPoly(aruco_mask, [pts], (255,))
    
    # subtract aruco areas from zone mask
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(aruco_mask))
    
    # convert to grayscale and apply heavier blur for less sharp edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 3)  # much heavier blur
    
    # for dark objects like black books, use intensity-based detection
    # approach 1: find dark regions
    _, dark_mask = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)  # dark areas become white
    dark_edges = cv2.Canny(dark_mask, 50, 150)
    
    # approach 2: less sensitive canny on original
    edges1 = cv2.Canny(blurred, 80, 200)  # higher thresholds for less sensitivity
    
    # approach 3: threshold-based for solid objects
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresh_edges = cv2.Canny(thresh, 100, 250)  # even higher for solid edges only
    
    # approach 4: adaptive threshold for varying lighting
    adaptive = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 5)
    adaptive_edges = cv2.Canny(adaptive, 100, 250)
    
    # combine all approaches, giving priority to dark object detection
    edges = cv2.bitwise_or(dark_edges, edges1)
    edges = cv2.bitwise_or(edges, thresh_edges)
    edges = cv2.bitwise_or(edges, adaptive_edges)
    
    print(f"Edge detection: dark={np.sum(dark_edges > 0)}, standard={np.sum(edges1 > 0)}, threshold={np.sum(thresh_edges > 0)}, adaptive={np.sum(adaptive_edges > 0)}")
    
    # create edge visualization on white canvas
    edge_canvas = np.ones((frame.shape[0], frame.shape[1], 3), dtype=np.uint8) * 255  # white background
    
    # draw different edge types in different colors
    edge_canvas[dark_edges > 0] = [255, 0, 0]  # red for dark object edges
    edge_canvas[edges1 > 0] = [0, 255, 0]  # green for standard edges  
    edge_canvas[thresh_edges > 0] = [0, 0, 255]  # blue for threshold edges
    edge_canvas[adaptive_edges > 0] = [255, 0, 255]  # magenta for adaptive edges
    
    # draw zone boundaries in black
    cv2.polylines(edge_canvas, [zone_pts], True, (0, 0, 0), 2)
    
    # draw aruco exclusion areas in gray
    edge_canvas[aruco_mask > 0] = [128, 128, 128]
    
    # show the edge detection canvas
    edge_display = cv2.resize(edge_canvas, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_LANCZOS4)
    cv2.imshow('Edge Detection', edge_display)
    
    # stronger morphological operations to connect edges and reduce noise
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)  # remove noise
    
    # apply zone mask to edges
    edges_masked = cv2.bitwise_and(edges, mask)
    
    # find contours
    contours, _ = cv2.findContours(edges_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} total contours")
    
    # also draw contours on the edge canvas for visualization
    cv2.drawContours(edge_canvas, contours, -1, (0, 0, 0), 1)  # black contour outlines
    
    current_obstacles = []
    obstacle_id = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        print(f"Contour {obstacle_id}: area={area:.0f}")
        
        if area < 1500:  # much higher threshold for substantial objects only
            print(f"  Filtered: area too small ({area:.0f} < 1500)")
            continue
            
        # get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        print(f"  Dimensions: {w}x{h}, aspect={aspect_ratio:.2f}")
        
        # filter out slivers by minimum width and height
        if w < 20 or h < 20:  # minimum 20 pixels in both dimensions
            print(f"  Filtered: too thin/narrow ({w}x{h}, need at least 20x20)")
            continue
        
        # filter out very elongated shapes (slivers)
        if aspect_ratio > 5 or aspect_ratio < 0.2:  # more restrictive aspect ratio
            print(f"  Filtered: too elongated ({aspect_ratio:.2f})")
            continue
        
        # calculate solidity (area/convex_hull_area) to filter out text-like shapes
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        print(f"  Solidity: {solidity:.2f}")
        
        # filter out non-solid shapes (like text which has low solidity)
        if solidity < 0.7:  # stricter solidity for more solid objects
            print(f"  Filtered: low solidity ({solidity:.2f} < 0.7) - likely fragmented")
            continue
        
        # calculate zone dimensions for relative filtering
        zone_area = cv2.contourArea(np.array(zone['corners'], dtype=np.int32))
        zone_width = max(zone['corners'], key=lambda p: p[0])[0] - min(zone['corners'], key=lambda p: p[0])[0]
        zone_height = max(zone['corners'], key=lambda p: p[1])[1] - min(zone['corners'], key=lambda p: p[1])[1]
        
        # reasonable size filtering
        if area > zone_area * 0.3:  # allow up to 30% of zone
            print(f"  Filtered: too large ({area:.0f} > {zone_area * 0.3:.0f})")
            continue
        
        # simple polygon approximation
        epsilon = 0.015 * cv2.arcLength(contour, True)  # slightly more aggressive to smooth
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # reasonable vertex count for clean shapes
        if len(approx) < 4 or len(approx) > 15:  # at least 4 vertices for solid objects
            print(f"  Filtered: vertex count ({len(approx)} vertices, need 4-15)")
            continue
        
        print(f"  ACCEPTED: area={area:.0f}, dims={w}x{h}, aspect={aspect_ratio:.2f}, solidity={solidity:.2f}, vertices={len(approx)}")
        
        # draw accepted contour in bright yellow on edge canvas
        cv2.drawContours(edge_canvas, [contour], -1, (0, 255, 255), 3)
        
        # convert vertices to mm coordinates
        vertices_mm = []
        for vertex in approx:
            px_x, px_y = vertex[0]
            mm_x = (px_x - bl_px[0]) / pixelsPerMm
            mm_y = (bl_px[1] - px_y) / pixelsPerMm
            vertices_mm.append((mm_x, mm_y))
        
        obstacle = Obstacle(obstacle_id, vertices_mm)
        current_obstacles.append(obstacle)
        obstacle_id += 1
    
    # add current detection to history
    obstacle_history.append(current_obstacles)
    if len(obstacle_history) > HISTORY_SIZE:
        obstacle_history.pop(0)
    
    # TEMPORARY: return current obstacles without stability filtering for debugging
    if current_obstacles:
        print(f"Found {len(current_obstacles)} obstacles (bypassing stability filter)")
    return current_obstacles
    
    # DISABLED: return stable obstacles (appear in at least STABILITY_THRESHOLD frames)
    # if len(obstacle_history) < STABILITY_THRESHOLD:
    #     return current_obstacles  # not enough history yet
    # 
    # stable_obstacles = []
    # for current_obs in current_obstacles:
    #     stability_count = 0
    #     for frame_obstacles in obstacle_history:
    #         for hist_obs in frame_obstacles:
    #             if _obstacles_similar(current_obs, hist_obs):
    #                 stability_count += 1
    #                 break
    #     
    #     if stability_count >= STABILITY_THRESHOLD:
    #         stable_obstacles.append(current_obs)
    # 
    # return stable_obstacles

def _obstacles_similar(obs1, obs2, threshold=30.0):  # tighter threshold for more stability
    """check if two obstacles are similar based on centroid distance and area."""
    if len(obs1.vertices) == 0 or len(obs2.vertices) == 0:
        return False
    
    # calculate centroids
    centroid1 = np.mean(obs1.vertices, axis=0)
    centroid2 = np.mean(obs2.vertices, axis=0)
    
    # calculate distance
    distance = np.linalg.norm(np.array(centroid1) - np.array(centroid2))
    
    # also check area similarity for better matching
    area1 = _calculate_polygon_area(obs1.vertices)
    area2 = _calculate_polygon_area(obs2.vertices)
    area_ratio = min(area1, area2) / max(area1, area2) if max(area1, area2) > 0 else 0
    
    return distance < threshold and area_ratio > 0.7  # must be similar in position and size

def _calculate_polygon_area(vertices):
    """calculate area of polygon using shoelace formula."""
    if len(vertices) < 3:
        return 0
    
    area = 0
    for i in range(len(vertices)):
        j = (i + 1) % len(vertices)
        area += vertices[i][0] * vertices[j][1]
        area -= vertices[j][0] * vertices[i][1]
    
    return abs(area) / 2

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