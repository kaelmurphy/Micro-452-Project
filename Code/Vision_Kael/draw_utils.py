import cv2
import numpy as np

def drawOperatingZone(frame, zone, color=(0, 255, 255)):
    """
    draws operating rectangle optional mid-edge markers, uses corners 0,2,4,6 to form polygon,
    adds fill to visualize boundaries
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


def drawRobotGoal(frame, centers, robotId=8, goalIds=None,
                  robotColor=(255, 200, 0), goalColor=(34, 139, 34),
                  radius=12, thickness=3, fontScale=0.8):
    """
    draw robot/goal outlines and labels.
    if corners are provided the function draws polygon outlines that match the marker borders.
    """
    if goalIds is None:
        goalIds = [9]

    out = frame.copy()

    for cid, val in centers.items():
        drawn = False
        # try corners -> polygon
        try:
            arr = np.asarray(val)
            if arr.shape == (4, 2):
                pts = arr.reshape((-1, 1, 2)).astype(int)
                if cid == robotId:
                    cv2.polylines(out, [pts], isClosed=True, color=robotColor, thickness=thickness, lineType=cv2.LINE_AA)
                    tx, ty = int(arr[1, 0]) + 6, int(arr[1, 1]) - 6
                    cv2.putText(out, "robot", (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, fontScale, robotColor, 2, cv2.LINE_AA)
                    try:
                        cxF, cyF = arr[:, 0].mean(), arr[:, 1].mean()
                        cx, cy = int(cxF), int(cyF)
                        topMidX = int((arr[0, 0] + arr[1, 0]) / 2)
                        topMidY = int((arr[0, 1] + arr[1, 1]) / 2)
                        cv2.line(out, (cx, cy), (topMidX, topMidY), (0, 0, 255), 2, cv2.LINE_AA)
                    except Exception:
                        pass
                    drawn = True
                elif cid in goalIds:
                    cv2.polylines(out, [pts], isClosed=True, color=goalColor, thickness=thickness, lineType=cv2.LINE_AA)
                    tx, ty = int(arr[1, 0]) + 6, int(arr[1, 1]) - 6
                    cv2.putText(out, "goal", (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, fontScale, goalColor, 2, cv2.LINE_AA)
                    drawn = True
        except Exception:
            drawn = False

        if drawn:
            continue

    return out