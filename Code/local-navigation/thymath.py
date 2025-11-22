import numpy as np

def wrap(radians: float) -> float:
    return (radians + np.pi) % (2 * np.pi) - np.pi

def split_int32(x: int) -> tuple[int, int]:
    x &= 0xFFFFFFFF
    lo = x & 0xFFFF
    if lo >= 0x8000:
        lo -= 0x10000
    hi = (x >> 16) & 0xFFFF
    if hi >= 0x8000:
        hi -= 0x10000
    return hi, lo
    
def combine_int32(hi: int, lo: int) -> int:
    hi &= 0xFFFF
    lo &= 0xFFFF
    x = (hi << 16) | lo
    if x >= 0x80000000:
        x -= 0x100000000
    return x

def cross2d(a: np.ndarray, b: np.ndarray):
    return a[0] * b[1] - a[1] * b[0]

def trajectory_direction(x: float, y: float, path: np.ndarray) -> int:
    if path.shape[0] >= 3:
        p0 = np.array([x, y])
        p1 = path[0]
        p2 = path[1]
        return -1 if cross2d(p1 - p0, p2 - p1) < 0 else 1
    else:
        return 1

def segments_intersection_distance(s1: np.ndarray, s2: np.ndarray) -> float | None:
    r = s1[1] - s1[0]
    s = s2[1] - s2[0]
    r_cross_s = cross2d(r, s)
    if abs(r_cross_s) < 1e-9:
        return None
    q_minus_p = s2[0] - s1[0]
    t = cross2d(q_minus_p, s) / r_cross_s
    u = cross2d(q_minus_p, r) / r_cross_s
    if 0 <= t <= 1 and 0 <= u <= 1:
        seg_len = np.linalg.norm(r)
        if seg_len < 1e-12:
            return None
        return t * seg_len
    return None

def path_intersection_distance(path: np.ndarray, segment: np.ndarray) -> tuple[float | None, int | None]:
    for i in range(len(path) - 1):
        pathSegment = path[i:(i + 2)]
        d = segments_intersection_distance(segment, pathSegment)
        if d is not None:
            return d, i
    return None, None
