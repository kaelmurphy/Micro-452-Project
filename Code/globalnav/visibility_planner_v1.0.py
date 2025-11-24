import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from shapely.geometry import Polygon, LineString
import math
import heapq
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from typing import Dict, List, Tuple


# 1. Helpers: orientation (CCW / CW)
def signed_area(ring):
    # shoelace formula; positive => CCW, negative => CW
    a = 0.0
    for (x1,y1),(x2,y2) in zip(ring, ring[1:]+ring[:1]):
        a += x1*y2 - x2*y1
    return 0.5 * a

def is_ccw(ring):
    return signed_area(ring) > 0

def ensure_ccw(ring):
    return ring if is_ccw(ring) else list(reversed(ring))

# 2. Parsing the input
def _df_from_input(map_array: np.ndarray) -> pd.DataFrame:
    """
    Convert input NumPy array (or DataFrame) to a clean DataFrame with
    columns: id, type, x, y.
    """
    if isinstance(map_array, pd.DataFrame):
        df = map_array.copy()
    else:
        # Expecting shape (N, 4): [id, type, x, y]
        if not isinstance(map_array, np.ndarray):
            raise TypeError("map_array must be a NumPy array or a pandas DataFrame.")
        if map_array.ndim != 2 or map_array.shape[1] < 4:
            raise ValueError("map_array must have shape (N, 4) with columns [id, type, x, y].")

        df = pd.DataFrame(map_array, columns=["id", "type", "x", "y"])

    # Normalize column names
    df.columns = [c.lower().strip() for c in df.columns]

    # Enforce dtypes
    df["id"] = df["id"].astype(str)
    df["type"] = df["type"].astype(str)
    df["x"] = pd.to_numeric(df["x"], errors="coerce")
    df["y"] = pd.to_numeric(df["y"], errors="coerce")

    # Drop rows without coordinates
    df = df.dropna(subset=["x", "y"])

    return df

def _build_polygons_and_special_points(
    df: pd.DataFrame
) -> Tuple[Dict[str, List[Tuple[float, float]]], Dict[str, Tuple[float, float]]]:
    """
    Build:
      - polygons: dict pid -> [(x, y), ...]
      - special_points: {'start': (sx, sy), 'goal': (gx, gy), ...}
    """

    # Polygons: only groups with >= 3 vertices become polygons
    polygons = {
        pid: [(float(x), float(y)) for x, y in g[["x", "y"]].to_numpy()]
        for pid, g in df.groupby("id")
        if len(g) >= 3
    }

    if "poly0" not in polygons:
        raise ValueError("Outer boundary polygon 'poly0' is required but not found in input.")

    # Special points (start, goal, waypoint, etc.)
    mask = df["type"].str.lower().isin(["start", "goal", "waypoint"])
    special_points = {
        row["type"].lower(): (float(row["x"]), float(row["y"]))
        for _, row in df[mask].iterrows()
    }

    if "start" not in special_points or "goal" not in special_points:
        raise ValueError("Both a START and a GOAL point are required in the input (type column).")

    # Force polygons to CCW + track which actually got flipped
    flipped = []
    polygons_ccw = {}
    for pid, pts in polygons.items():
        ccw_pts = ensure_ccw(pts)
        if ccw_pts != pts:
            flipped.append(pid)
        polygons_ccw[pid] = ccw_pts

    # Print summary
    if len(flipped) == 0:
        print("All polygons are already counter-clockwise (CCW).")
    else:
        print("The following polygons were clockwise (CW) and have been flipped to CCW:")
        for pid in flipped:
            print(f"  - {pid}")

    print("Number of polygons:", len(polygons_ccw))
    print("polygons keys:", list(polygons_ccw.keys()))
    print("special points:", special_points)


    return polygons_ccw, special_points




# 3. Polygon inflation
def inflate_polygons(polygons_ccw, epsilon, shrink_outer=True):
    """
    polygons_ccw: dict pid -> [(x,y), ...]  (already CCW for the original shapes)
    epsilon: inflation radius (cm)
    shrink_outer: (currently ignored) outer boundary poly0 is kept unchanged
    
    returns: new dict pid -> [(x,y), ...] for inflated/shrunk polygons,
             each ring forced to CCW using ensure_ccw().
    """
    inflated = {}

    for pid, ring in polygons_ccw.items():
        poly = Polygon(ring)

        if pid == "poly0":
            # Do NOT inflate or shrink outer boundary: keep as-is
            ring_ccw = ensure_ccw(ring)
            inflated[pid] = ring_ccw
            continue

        # 1) Inflate obstacles (all polygons except poly0)
        buffered = poly.buffer(epsilon, join_style=2, resolution=1)

        # 2) Handle possible MultiPolygon (rare for nice convex shapes)
        if buffered.geom_type == "MultiPolygon":
            parts = list(buffered.geoms)
            buffered = max(parts, key=lambda p: p.area)

        # 3) Extract exterior coordinates
        xs, ys = buffered.exterior.coords.xy
        ring_raw = list(zip(xs, ys))[:-1]  # drop last point == first point

        # 4) Enforce CCW using your own helper
        ring_ccw = ensure_ccw(ring_raw)

        inflated[pid] = ring_ccw

    return inflated


# 4. Building graph nodes from polygons
def build_nodes_from_polygons(polygons_ccw, special_points):
    """
    polygons_ccw: dict pid -> [(x,y), ...], including 'poly0'
    special_points: {'start': (sx,sy), 'goal': (gx,gy)}

    Returns:
        node_coords: list of (x,y)
        node_labels: list of labels for each node (str)
        start_idx: index of start node
        goal_idx: index of goal node
    """
    node_coords = []
    node_labels = []

    # 1) obstacle vertices (all polygons except poly0)
    for pid, ring in polygons_ccw.items():
        if pid == "poly0":
            continue  # skip outer boundary as nodes

        for k, (x, y) in enumerate(ring):
            node_coords.append((float(x), float(y)))
            node_labels.append(f"{pid}_{k}")  # simple label; can change later

    # 2) start node
    sx, sy = special_points["start"]
    start_idx = len(node_coords)
    node_coords.append((float(sx), float(sy)))
    node_labels.append("start")

    # 3) goal node
    gx, gy = special_points["goal"]
    goal_idx = len(node_coords)
    node_coords.append((float(gx), float(gy)))
    node_labels.append("goal")

    N = len(node_coords)
    print("Total nodes:", N)
    print("start_idx:", start_idx, "goal_idx:", goal_idx)


    return node_coords, node_labels, start_idx, goal_idx 

# 5. Visibility graph construction
def segment_visible(p, q, world_poly, obstacle_polys):
    """
    Check if segment p–q is valid:
      - stays within world_poly
      - does not cross obstacle interiors
      - touching at vertices/edges is allowed
    """
    seg = LineString([p, q])

    # 1) Stay inside/ on the boundary of the world
    if not (seg.within(world_poly) or seg.touches(world_poly)):
        return False

    # 2) Check obstacles
    for poly in obstacle_polys:
        # if segment properly crosses interior: not allowed
        if seg.crosses(poly):
            return False
        # if entire segment lies inside obstacle: not allowed
        if seg.within(poly) or poly.contains(seg):
            return False
        # seg.touches(poly): allowed (your chosen policy)

    return True


def euclidean_distance(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return math.hypot(dx, dy)


def build_neighbors(node_coords, world_poly, obstacle_polys):
    """
    Build visibility graph adjacency list.

    node_coords: list of (x,y)
    world_poly: shapely Polygon for outer boundary
    obstacle_polys: list of shapely Polygons (inflated or original)

    Returns:
        neighbors: list-of-lists adjacency: neighbors[i] = [(j, weight_ij), ...]
    """
    N = len(node_coords)
    neighbors = [[] for _ in range(N)]

    for i in range(N):
        p = node_coords[i]
        for j in range(i + 1, N):
            q = node_coords[j]

            if segment_visible(p, q, world_poly, obstacle_polys):
                d = euclidean_distance(p, q)
                neighbors[i].append((j, d))
                neighbors[j].append((i, d))

    return neighbors

#7. A* search on the visibility graph
def heuristic(i, goal_idx, node_coords):
    """Euclidean distance from node i to goal."""
    x1, y1 = node_coords[i]
    x2, y2 = node_coords[goal_idx]
    return math.hypot(x1 - x2, y1 - y2)

def astar(neighbors, node_coords, start_idx, goal_idx):
    """
    Run A* on a visibility graph.

    neighbors: adjacency list, neighbors[i] = [(j, weight_ij), ...]
    node_coords: list of (x, y) for each node index
    start_idx, goal_idx: integer node indices

    Returns:
        path_indices: [start_idx, ..., goal_idx] or None if no path
        explored: set of visited node indices (for debugging/plotting)
        operation_count: how many neighbor relaxations we did
    """
    # Cost from start to each node (g-cost)
    g_cost = {start_idx: 0.0}

    # To reconstruct path: child -> parent
    came_from = {}

    # Explored set (closed set)
    explored = set()

    # Priority queue: (f_cost, g_cost, node_idx)
    open_set = []
    start_h = heuristic(start_idx, goal_idx, node_coords)
    heapq.heappush(open_set, (start_h, 0.0, start_idx))

    operation_count = 0

    while open_set:
        f_curr, g_curr, current = heapq.heappop(open_set)

        # If we've already processed this node with a better g, skip
        if current in explored:
            continue

        explored.add(current)

        # Goal check
        if current == goal_idx:
            break

        # Relax all neighbors
        for neighbor, weight in neighbors[current]:
            tentative_g = g_curr + weight

            # If this path to neighbor is better, record it
            if (neighbor not in g_cost) or (tentative_g < g_cost[neighbor]):
                g_cost[neighbor] = tentative_g
                came_from[neighbor] = current
                h = heuristic(neighbor, goal_idx, node_coords)
                f = tentative_g + h
                heapq.heappush(open_set, (f, tentative_g, neighbor))
                operation_count += 1

    # Reconstruct path if goal reached
    if goal_idx not in came_from and start_idx != goal_idx:
        # No path found
        return None, explored, operation_count

    path_indices = [goal_idx]
    current = goal_idx
    while current != start_idx:
        current = came_from[current]
        path_indices.append(current)
    path_indices.reverse()

    if path_indices is None:
        print("No path found.")
    else:
        print("A* operation count:", operation_count)
        print("Number of explored nodes:", len(explored))
        print("Path node indices:", path_indices)

    return path_indices, explored, operation_count


def compute_global_path(
    map_array: np.ndarray,
    epsilon_mm: float = 20.0,
    ) -> np.ndarray:
    
    # 1) Build DataFrame and scale coordinates
    df = _df_from_input(map_array)

    # 2) Build (CCW) polygons and special points
    polygons_ccw_raw, special_points = _build_polygons_and_special_points(df)

    # 3) Inflate obstacles
    EPSILON = float(epsilon_mm)

    polygons_ccw_eps = inflate_polygons(polygons_ccw_raw, EPSILON, shrink_outer=True)

    # 4) Build shapely polygons for inflated world
    world_poly_eps = Polygon(polygons_ccw_eps["poly0"])
    obstacle_polys_eps = [
        Polygon(ring) for pid, ring in polygons_ccw_eps.items() if pid != "poly0"
    ]

    # 5) Build nodes for inflated world
    node_coords_eps, node_labels_eps, start_idx_eps, goal_idx_eps = build_nodes_from_polygons(polygons_ccw_eps, special_points)

    # 6) Build neighbors (visibility graph)
    neighbors_eps = build_neighbors(node_coords_eps, world_poly_eps, obstacle_polys_eps)
    
    print("\n=== NEIGHBOR COUNTS ===")
    for i, nbrs in enumerate(neighbors_eps):
        print(f"Node {i:2d} ({node_labels_eps[i]:8s}) has {len(nbrs)} neighbors")

    # 7) Run A*
    path_indices, explored_nodes, op_count = astar(
        neighbors_eps, node_coords_eps, start_idx_eps, goal_idx_eps
    )

    if path_indices is None:
        raise RuntimeError("No path found by A* for the given map and inflation radius.")

    # Optional: debug path labels & coordinates
    print("Path labels:", [node_labels_eps[i] for i in path_indices])
    print("Path coordinates:", [node_coords_eps[i] for i in path_indices])

    # 8) Turn into NumPy array: global_path
    global_path = np.array([node_coords_eps[i] for i in path_indices], dtype=float)

    return global_path