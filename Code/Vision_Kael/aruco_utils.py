import cv2
import numpy as np

def detectAruco(frame, dictName="DICT_4X4_50", draw=True):
    '''
    detect ArUco markers, returns (ids, centers, cornersMap, annotatedFrame)
    '''
    # check if ArUco module available
    if not hasattr(cv2, "aruco"):
        return [], {}, {}, frame
    # validate dictionary name
    if not hasattr(cv2.aruco, dictName):
        dictName = "DICT_4X4_50"
    # setup ArUco detector
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictName))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    # detect markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    annotated = frame.copy()
    centers = {}
    cornersMap = {}
    idList = []
    # process detected markers
    if ids is not None and len(ids) > 0:
        idList = ids.flatten().tolist()
        if draw:
            cv2.aruco.drawDetectedMarkers(annotated, corners)
        for i, cid in enumerate(idList):
            # store corner points
            pts = corners[i][0]
            cornersMap[cid] = pts.astype(int)
            # compute center
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            centers[cid] = (cx, cy)
            # draw center point
            cv2.circle(annotated, (cx, cy), 6, (255, 200, 0), -1)
            # assign label and color based on marker ID
            label = f"id: {cid}" if cid < 8 else ("robot" if cid == 8 else "goal")
            color = (255, 200, 0) if cid < 8 else ((255, 200, 200) if cid == 8 else (255, 0, 200))
            cv2.putText(annotated, label, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
    return idList, centers, cornersMap, annotated


def buildOperatingZone(centers):
    '''
    build operating zone from corner markers 0-3 (TL, TR, BR, BL)
    '''
    # markers 0-3 define zone corners
    required = [0, 1, 2, 3]
    missing = [i for i in required if i not in centers]
    # build zone dictionary
    zone = {'isComplete': len(missing) == 0, 'missing': missing, 'corners': []}
    # add corners if all markers present: [TL, TR, BR, BL]
    if all(i in centers for i in required):
        zone['corners'] = [centers[0], centers[1], centers[2], centers[3]]
    return zone