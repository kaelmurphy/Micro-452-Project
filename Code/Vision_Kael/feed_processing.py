import cv2
import numpy as np

def toGray(frame):
    # convert frame to grayscale
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def detectEdges(frame, low=25, high=80, blur=3):
    # convert to grayscale
    g = toGray(frame)
    # apply contrast to make edges more visible
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    g = clahe.apply(g)
    # median blur to clean up noise
    if blur and blur > 0:
        k = blur | 1
        g = cv2.medianBlur(g, k)
    # canny edge detection
    return cv2.Canny(g, low, high)

def overlayEdges(frame, edges):
    # overlay detected edges in white on original frame
    out = frame.copy()
    out[edges > 0] = (255, 255, 255)
    return out

def detectAruco(frame, dictName="DICT_4X4_50", draw=True):
    """
    returns (ids:list[int], centers:dict[id]=(x,y), annotatedFrame)
    finds aruco markers and labels them once in sky blue
    """
    # skip if aruco module missing
    if not hasattr(cv2, "aruco"):
        return [], {}, frame

    # get dictionary type, fallback to 4x4_50
    if not hasattr(cv2.aruco, dictName):
        dictName = "DICT_4X4_50"
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictName))

    # setup detector
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)

    # detect markers on grayscale frame
    gray = toGray(frame)
    corners, ids, _ = detector.detectMarkers(gray)

    # copy frame to draw on
    annotated = frame.copy()
    # store marker centers
    centers = {}
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
            # calculate marker center
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            centers[cid] = (cx, cy)
            # draw center dot
            cv2.circle(annotated, (cx, cy), 6, (255, 200, 0), -1)
            # write id label
            cv2.putText(
                annotated,
                f"id: {cid}",
                (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 200, 0),
                2,
                cv2.LINE_AA
            )

    return idList, centers, annotated

def buildOperatingZone(centers):
    """
    centers: dict[id] = (x,y) from detectAruco
    returns dict defining operating rectangle using markers 0 through 7
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

def drawOperatingZone(frame, zone, color=(0, 255, 255)):
    """
    draws the operating rectangle and optional mid-edge markers, uses corners 0,2,4,6 to form a polygon,
    adds a fill to visualize boundaries
    """
    # check if enough corner points exist
    if not zone['corners']:
        return frame

    # extract points in correct order
    tl, tr, br, bl = zone['corners']
    pts = np.array([tl, tr, br, bl], dtype=np.int32)

    # copy frame to draw overlays
    out = frame.copy()

    # draw fill area
    overlay = out.copy()
    cv2.fillPoly(overlay, [pts], color)
    cv2.addWeighted(overlay, 0.15, out, 0.85, 0, out)

    # draw thick border
    cv2.polylines(out, [pts], isClosed=True, color=color, thickness=3, lineType=cv2.LINE_AA)

    # draw edge midpoint dots if they exist
    for midId in [1,3,5,7]:
        if midId in zone['edgeMids']:
            x, y = zone['edgeMids'][midId]
            cv2.circle(out, (int(x), int(y)), 6, color, -1, cv2.LINE_AA)
    
    return out
