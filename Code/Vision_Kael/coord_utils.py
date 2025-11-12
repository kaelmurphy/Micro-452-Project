import numpy as np

# persistent smoothing storage for stable coordinates across frames
_prevState = {}
# smoothing factor for exponential moving average (new_value_weight)
# lower -> more smoothing (0.0..1.0)
_smoothingAlpha = 0.4

def _smooth_tuple(key, name, val, alpha=_smoothingAlpha):
    """
    EMA-smooth a 2-tuple (x,y). Stores previous in _prevState under key/name.
    """
    if val is None:
        return None
    prev = _prevState.get(key, {}).get(name)
    cur = (float(val[0]), float(val[1]))
    if prev is None:
        _prevState.setdefault(key, {})[name] = cur
        return cur
    # element-wise EMA
    sm = (alpha * cur[0] + (1 - alpha) * prev[0], alpha * cur[1] + (1 - alpha) * prev[1])
    _prevState.setdefault(key, {})[name] = sm
    return sm

def _smooth_angle(key, name, angle, alpha=_smoothingAlpha):
    """
    smooth angle (radians) by EMA on unit-vector components.
    """
    if angle is None:
        return None
    prev_vec = _prevState.get(key, {}).get(name + "_vec")
    cur_vec = np.array([np.cos(angle), np.sin(angle)], dtype=float)
    if prev_vec is None:
        _prevState.setdefault(key, {})[name + "_vec"] = cur_vec
        _prevState.setdefault(key, {})[name] = float(angle)
        return float(angle)
    sm_vec = alpha * cur_vec + (1 - alpha) * prev_vec
    # store
    _prevState.setdefault(key, {})[name + "_vec"] = sm_vec
    ang = float(np.arctan2(sm_vec[1], sm_vec[0]))
    _prevState.setdefault(key, {})[name] = ang
    return ang

def _as_xy(val):
    """
    normalize value to an (x,y) pair of floats or return None.
    """
    if val is None:
        return None
    try:
        x = float(val[0])
        y = float(val[1])
        return (x, y)
    except Exception:
        return None


def worldToZone(point, zone):
    """
    Map a world-image point (x,y) into zone-local coordinates (u,v).

    zone: dict with 'corners' == [tl, tr, br, bl] in image coordinates (or world coords).
    Returns (u, v) where u and v are coordinates in the basis (tl->tr, tl->bl).
    If zone corners are not available, returns None.
    """
    if not zone or 'corners' not in zone or not zone['corners']:
        return None
    tl, tr, br, bl = zone['corners']
    tl = np.array(tl, dtype=float)
    tr = np.array(tr, dtype=float)
    bl = np.array(bl, dtype=float)
    p = np.array(point, dtype=float)

    # basis vectors
    ex = tr - tl
    ey = bl - tl

    # build 2x2 matrix [ex ey]
    M = np.column_stack((ex, ey))
    try:
        coeffs = np.linalg.solve(M, (p - tl))
    except np.linalg.LinAlgError:
        return None

    u, v = float(coeffs[0]), float(coeffs[1])
    return (u, v)


def robotWorldPose(centers, cornersMap=None, robotId=8):
    """
    compute robot center (x,y) in world/image coordinates and orientation theta (radians)
    using marker corners if available. Returns (cx, cy, theta) or (None, None, None)
    if robot not found.

    theta is angle of vector from marker center to top-middle edge
    in image pixel coordinates, in range [-pi, pi].
    """
    if robotId not in centers:
        return (None, None, None)

    # center from centers dict
    c = centers[robotId]
    try:
        cx, cy = int(c[0]), int(c[1])
    except Exception:
        return (None, None, None)

    # compute orientation using corners if available
    if cornersMap and robotId in cornersMap:
        arr = np.asarray(cornersMap[robotId], dtype=float)
        # top-left is arr[0], top-right arr[1]
        topMid = np.array([(arr[0,0] + arr[1,0]) / 2.0, (arr[0,1] + arr[1,1]) / 2.0])
        vec = topMid - np.array([cx, cy], dtype=float)
        theta = float(np.arctan2(vec[1], vec[0]))
        return (cx, cy, theta)

    # fallback: no corners available, we can't determine heading reliably
    return (cx, cy, None)