import cv2
import numpy as np

def detectAruco(frame, dictName="DICT_4X4_50"):
    if not hasattr(cv2, "aruco"):
        return [], {}, {}, frame

    dict_attr = getattr(cv2.aruco, dictName, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
    detector = cv2.aruco.ArucoDetector(dictionary)

    # Use full-resolution grayscale image for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Optionally, apply contrast enhancement if needed (commented out for now)
    # gray = cv2.equalizeHist(gray)
    corners, ids, _ = detector.detectMarkers(gray)

    centers = {}
    cornersMap = {}
    idList = []

    if ids is not None:
        idList = ids.flatten().tolist()
        for i, marker_id in enumerate(idList):
            pts = corners[i][0]  # No scaling needed
            cornersMap[marker_id] = pts.astype(int)
            center = pts.mean(axis=0).astype(int)
            centers[marker_id] = tuple(center)

    return idList, centers, cornersMap, frame  # Skip annotated frame for speed

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