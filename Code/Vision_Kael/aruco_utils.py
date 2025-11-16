import cv2
import numpy as np

def detectAruco(frame, dictName="DICT_4X4_50"):
    if not hasattr(cv2, "aruco"):
        return [], {}, {}, frame
    
    dict_attr = getattr(cv2.aruco, dictName, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
    detector = cv2.aruco.ArucoDetector(dictionary)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    annotated = frame.copy()
    centers = {}
    cornersMap = {}
    idList = []
    
    if ids is not None:
        idList = ids.flatten().tolist()
        cv2.aruco.drawDetectedMarkers(annotated, corners)
        
        for i, marker_id in enumerate(idList):
            pts = corners[i][0]
            cornersMap[marker_id] = pts.astype(int)
            center = pts.mean(axis=0).astype(int)
            centers[marker_id] = tuple(center)
            
            cv2.circle(annotated, tuple(center), 6, (255, 200, 0), -1)
            
            if marker_id < 8:
                label, color = f"id: {marker_id}", (255, 200, 0)
            elif marker_id == 8:
                label, color = "robot", (255, 200, 200)
            else:
                label, color = "goal", (255, 0, 200)
                
            cv2.putText(annotated, label, (center[0] + 10, center[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    
    return idList, centers, cornersMap, annotated

def buildOperatingZone(centers):
    required = [0, 1, 2, 3]
    missing = [i for i in required if i not in centers]
    # return corners in order [bl, br, tr, tl] as expected by feed_processing
    corner_order = [3, 2, 1, 0]  # [bl, br, tr, tl]
    return {
        'isComplete': len(missing) == 0,
        'missing': missing,
        'corners': [centers[i] for i in corner_order if i in centers]
    }