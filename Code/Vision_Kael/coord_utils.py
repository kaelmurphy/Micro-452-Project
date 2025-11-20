import numpy as np
import cv2

def asXy(val):
    if val is None:
        return None
    try:
        return (float(val[0]), float(val[1]))
    except:
        return None

def robotWorldPose(centers, cornersMap, robotId):
    if robotId not in centers:
        return (None, None, None)
    
    try:
        cx, cy = int(centers[robotId][0]), int(centers[robotId][1])
    except:
        return (None, None, None)
    
    if robotId in cornersMap:
        corners = cornersMap[robotId].astype(float)
        topMid = (corners[0] + corners[1]) / 2.0
        dx = topMid[0] - cx
        dy = topMid[1] - cy
        theta = float(np.arctan2(-dy, dx))
        return (cx, cy, theta)
    
    return (cx, cy, None)