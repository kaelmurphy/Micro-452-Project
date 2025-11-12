import cv2
import numpy as np


def detectAruco(frame, dictName="DICT_4X4_50", draw=True):
    """
    returns (ids:list[int], centers:dict[id]=(x,y), cornersMap, annotatedFrame)
    finds aruco markers and labels them once in blue
    """
    # skip if aruco module missing
    if not hasattr(cv2, "aruco"):
        # maintain consistent return shape (ids, centers, cornersMap, annotated)
        return [], {}, {}, frame

    # get dictionary type, fallback to 4x4_50
    if not hasattr(cv2.aruco, dictName):
        dictName = "DICT_4X4_50"
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictName))

    # setup detector
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)

    # detect markers on grayscale frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    # copy frame to draw on
    annotated = frame.copy()
    # store marker centers
    centers = {}
    # store marker corner points
    cornersMap = {}
    # store detected ids
    idList = []

    # if any markers found
    if ids is not None and len(ids) > 0:
        # flatten ids into list
        idList = ids.flatten().tolist()
        # draw marker outlines
        if draw:
            cv2.aruco.drawDetectedMarkers(annotated, corners)
        # loop through markers and add labels
        for i, cid in enumerate(idList):
            # get four corner points
            pts = corners[i][0]
            # save corners as integer tuples
            cornersMap[cid] = pts.astype(int)
            # calculate marker center
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            centers[cid] = (cx, cy)
            # draw center dot
            cv2.circle(annotated, (cx, cy), 6, (255, 200, 0), -1)
            # write id label
            if cid < 8:
                cv2.putText(annotated, f"id: {cid}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2, cv2.LINE_AA)
            elif cid == 8:
                cv2.putText(annotated, f"robot", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 200), 2, cv2.LINE_AA)
            else:
                cv2.putText(annotated, f"goal", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 200), 2, cv2.LINE_AA)

    return idList, centers, cornersMap, annotated


def buildOperatingZone(centers):
    """
    centers: dict[id] = (x,y).
    returns dict defining operating rectangle using markers 0 through 7:
    0 = top-left, 1 = top mid, 2 = top-right, 3 = right mid,
    4 = bottom-right, 5 = bottom mid, 6 = bottom-left, 7 = left mid
    """
    # list of required ids
    required = [0,1,2,3,4,5,6,7]
    # list missing ids
    missing = [i for i in required if i not in centers]

    # define output dictionary
    zone = {
        'isComplete': len(missing) == 0,
        'missing': missing,
        'corners': [],
        'edgeMids': {}
    }

    # add corners if all present
    cornerIds = [0, 2, 4, 6]
    if all(cid in centers for cid in cornerIds):
        tl = centers[0]
        tr = centers[2]
        br = centers[4]
        bl = centers[6]
        zone['corners'] = [tl, tr, br, bl]

    # add midpoints for edges if available
    for mid in [1,3,5,7]:
        if mid in centers:
            zone['edgeMids'][mid] = centers[mid]

    return zone