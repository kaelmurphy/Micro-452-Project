import cv2
import numpy as np

def drawOperatingZone(frame, zone, color=(0, 255, 255)):
    '''
    draw operating zone boundary with semi-transparent fill
    '''
    if not zone['corners']:
        return frame
    # prepare corner points
    pts = np.array(zone['corners'], dtype=np.int32)
    out = frame.copy()
    # draw semi-transparent fill
    overlay = out.copy()
    cv2.fillPoly(overlay, [pts], color)
    cv2.addWeighted(overlay, 0.15, out, 0.85, 0, out)
    # draw zone boundary
    cv2.polylines(out, [pts], True, color, 3, cv2.LINE_AA)
    return out


def drawRobotGoal(frame, centers, robotId=8, goalId=9):
    '''
    draw robot and goal markers with orientation line for robot
    '''
    out = frame.copy()
    # iterate through all detected markers
    for cid, val in centers.items():
        try:
            arr = np.asarray(val)
            # check if this is corner data (4 points)
            if arr.shape == (4, 2):
                pts = arr.reshape((-1, 1, 2)).astype(int)
                # draw robot marker
                if cid == robotId:
                    cv2.polylines(out, [pts], True, (255, 200, 0), 3, cv2.LINE_AA)
                    cv2.putText(out, "robot", (int(arr[1, 0]) + 6, int(arr[1, 1]) - 6), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 200, 0), 2, cv2.LINE_AA)
                    # draw orientation line from center to top edge
                    cx, cy = int(arr[:, 0].mean()), int(arr[:, 1].mean())
                    topMid = ((arr[0] + arr[1]) / 2).astype(int)
                    cv2.line(out, (cx, cy), tuple(topMid), (0, 0, 255), 2, cv2.LINE_AA)
                # draw goal marker
                elif cid == goalId:
                    cv2.polylines(out, [pts], True, (34, 139, 34), 3, cv2.LINE_AA)
                    cv2.putText(out, "goal", (int(arr[1, 0]) + 6, int(arr[1, 1]) - 6), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (34, 139, 34), 2, cv2.LINE_AA)
        except:
            pass
    return out