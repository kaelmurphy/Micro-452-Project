import numpy as np

prevState = {}
smoothingAlpha = 0.4

def smoothTuple(key, name, val):
    '''
    EMA-smooth tuple for stable coordinates across frames
    '''
    if val is None:
        return None
    # get previous smoothed value
    prev = prevState.get(key, {}).get(name)
    cur = (float(val[0]), float(val[1]))
    # no previous value, store and return current
    if prev is None:
        prevState.setdefault(key, {})[name] = cur
        return cur
    # apply exponential moving average to each component
    sm = (smoothingAlpha * cur[0] + (1 - smoothingAlpha) * prev[0], 
          smoothingAlpha * cur[1] + (1 - smoothingAlpha) * prev[1])
    prevState.setdefault(key, {})[name] = sm
    return sm

def smoothAngle(key, name, angle):
    '''
    smooth angle (degrees) using EMA on unit-vector components to avoid wrap-around issues
    '''
    if angle is None:
        return None
    # get previous smoothed unit vector
    prevVec = prevState.get(key, {}).get(name + "_vec")
    # convert angle to unit vector
    rad = angle * np.pi / 180.0
    curVec = np.array([np.cos(rad), np.sin(rad)])
    # no previous value, store and return current
    if prevVec is None:
        prevState.setdefault(key, {})[name + "_vec"] = curVec
        prevState.setdefault(key, {})[name] = float(angle)
        return float(angle)
    # smooth unit vector components
    smVec = smoothingAlpha * curVec + (1 - smoothingAlpha) * prevVec
    prevState.setdefault(key, {})[name + "_vec"] = smVec
    # convert smoothed unit vector back to angle in degrees
    ang = float(np.arctan2(smVec[1], smVec[0]) * 180.0 / np.pi)
    prevState.setdefault(key, {})[name] = ang
    return ang

def asXy(val):
    '''
    normalize value to (x,y) tuple of floats or return None
    '''
    if val is None:
        return None
    try:
        return (float(val[0]), float(val[1]))
    except:
        return None


def worldToZone(point, zone):
    '''
    map world point (x,y) in mm to zone-local coordinates (u,v) in [0,1] range using skew-tolerant basis vectors: origin at BL, x-axis along bottom edge, y-axis along left edge
    '''
    if not zone or 'corners' not in zone or not zone['corners']:
        return None
    # extract corners: [tl, tr, br, bl]
    tl, tr, br, bl = zone['corners']
    p = np.array(point, dtype=float)
    bl = np.array(bl, dtype=float)
    # basis vectors from bottom-left corner
    ex = np.array(br, dtype=float) - bl
    ey = np.array(tl, dtype=float) - bl
    try:
        # solve for coefficients: p = bl + u*ex + v*ey
        coeffs = np.linalg.solve(np.column_stack((ex, ey)), p - bl)
        return (float(coeffs[0]), float(coeffs[1]))
    except:
        return None


def robotWorldPose(centers, cornersMap, robotId):
    '''
    compute robot center (x,y) and orientation theta (radians) from marker corners
    '''
    if robotId not in centers:
        return (None, None, None)
    # extract center coordinates
    try:
        cx, cy = int(centers[robotId][0]), int(centers[robotId][1])
    except:
        return (None, None, None)
    # compute orientation from marker corners if available
    if cornersMap and robotId in cornersMap:
        arr = cornersMap[robotId].astype(float)
        # midpoint of top edge (between top-left and top-right corners)
        topMid = (arr[0] + arr[1]) / 2.0
        # angle from center to top midpoint
        theta = float(np.arctan2(topMid[1] - cy, topMid[0] - cx))
        return (cx, cy, theta)
    return (cx, cy, None)