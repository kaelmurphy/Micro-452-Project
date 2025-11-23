import numpy as np

def cross2d(a: np.ndarray, b: np.ndarray):
    return a[0] * b[1] - a[1] * b[0]

def trajectory_direction(path: np.ndarray) -> int:
    if path.shape[0] >= 3:
        return -1 if cross2d(path[1] - path[0], path[2] - path[1]) < 0 else 1
    else:
        return 1

def segments_intersection_point(s1: np.ndarray, s2: np.ndarray) -> np.ndarray | None:
    r = s1[1] - s1[0]
    s = s2[1] - s2[0]
    r_cross_s = cross2d(r, s)
    if abs(r_cross_s) < 1e-9:
        return None

    q_minus_p = s2[0] - s1[0]
    t = cross2d(q_minus_p, s) / r_cross_s
    u = cross2d(q_minus_p, r) / r_cross_s

    if 0 <= t <= 1 and 0 <= u <= 1:
        return s1[0] + t * r

    return None

def path_intersection_point(path: np.ndarray, segment: np.ndarray) -> tuple[np.ndarray | None, int | None]:
    for i in range(len(path) - 1):
        path_segment = path[i:(i + 2)]
        point = segments_intersection_point(segment, path_segment)
        if point is not None:
            return point, i
    return None, None
