"""
Visualization utilities for the global navigation module.

This file builds a Matplotlib figure showing:
- original vs inflated polygonal obstacles (configuration-space inflation),
- start/goal markers,
- the A* optimal path computed on the visibility graph,
- live-updated odometry and camera-based trajectories.

It is used by the dashboard to visualize the global plan and the robot’s motion
(see report: Global Navigation section, Visibility Graph + A*).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, FancyArrowPatch, Circle

from globalnav import compute_global_path_with_debug

# ---------------------------------------------------------
# GLOBAL VISUAL CONSTANTS
# ---------------------------------------------------------
ARROW_LENGTH = 100.0   # mm, visual length of odometry arrow
ROBOT_RADIUS = 30.0    # mm, radius of drawn robot/camera circles
TICK_STEP   = 100.0    # mm, grid spacing for axes

# ---------------------------------------------------------
# USER-DEFINED COLOR SYSTEM (ALL IN THIS FILE)
# ---------------------------------------------------------

COLOR_START = "#0077CC"   # blue
COLOR_GOAL  = "#CC0000"   # red

# Base colors for polygons
# 0: world boundary (poly0), 1..N: obstacles
POLYGON_BASE_COLORS = [
    "#CFCDCD",  # 0: world boundary (light grey)
    "#ff7f0e",  # 1: obstacle 1 (orange)
    "#2ca02c",  # 2: obstacle 2 (green)
    "#d62728",  # 3: obstacle 3 (red)
    "#9467bd",  # 4: obstacle 4 (purple)
]


def get_polygon_colors(num_polygons, base_colors):
    """
    num_polygons: number of polygons to color
    base_colors: user-defined list of hex colors

    Returns: list of colors for each polygon (length == num_polygons)
    """
    if num_polygons <= len(base_colors):
        return base_colors[:num_polygons]

    # If more polygons than colors: extend automatically
    extra_needed = num_polygons - len(base_colors)

    # Generate extra colors from matplotlib colormap
    cmap = plt.cm.get_cmap("tab20")
    extra_colors = [cmap(i / max(1, extra_needed)) for i in range(extra_needed)]

    # Convert RGBA → hex
    extra_colors = [mpl.colors.to_hex(c) for c in extra_colors]

    return base_colors + extra_colors


# ---------------------------------------------------------
# MAIN PLOTTING FUNCTION
# ---------------------------------------------------------

def setup_globalnav_plot(
    map_array,
    epsilon_mm: float = 20.0,
    show_neighbors: bool = False,
    initial_theta: float = 0.0,
    fig=None,
    ax=None,
):
    """
    Creates a Matplotlib window with:
      - Original vs inflated polygons
      - Start/Goal
      - A* optimal path
      - (Optional) visibility edges
      - An empty odometry trajectory line you can update live.

    Parameters
    ----------
    map_array : np.ndarray or DataFrame
        Must be compatible with globalnav._df_from_input
        (your demo uses: ["type", "id", "label", "x", "y"]).

    epsilon_mm : float
        Obstacle inflation radius in mm (or whatever units your
        coordinates are in).

    show_neighbors : bool
        If True, also draws the visibility graph edges.

    Returns
    -------
    global_path : np.ndarray
    debug : dict
    fig, ax : Matplotlib figure/axes
    odom_line : Line2D
    odom_arrow : FancyArrowPatch
    """

    # Get path + all debug info from globalnav
    global_path, debug = compute_global_path_with_debug(
        map_array, epsilon_mm=epsilon_mm
    )

    polygons_ccw_raw = debug["polygons_ccw_raw"]
    polygons_ccw_eps = debug["polygons_ccw_eps"]
    special_points   = debug["special_points"]
    node_coords      = debug["node_coords"]
    node_labels      = debug["node_labels"]
    neighbors        = debug["neighbors"]
    path_indices     = debug["path_indices"]
    EPSILON          = debug["epsilon_mm"]

    # ---------------------------------------------------------
    # Create or reuse figure / axes
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    ax.set_aspect("equal", adjustable="box")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # ---------------------------------------------------------
    # Polygon colors
    # ---------------------------------------------------------
    polygon_ids = list(polygons_ccw_eps.keys())
    num_polygons = len(polygon_ids)
    polygon_colors = get_polygon_colors(num_polygons, POLYGON_BASE_COLORS)

    # ---------------------------------------------------------
    # Bounds for axes (inflated polygons + start/goal)
    # ---------------------------------------------------------
    all_points = []
    for ring in polygons_ccw_eps.values():
        all_points.extend(ring)

    all_points.append(special_points["start"])
    all_points.append(special_points["goal"])

    bx, by = zip(*all_points)
    xmin, xmax = min(bx), max(bx)
    ymin, ymax = min(by), max(by)
    pad = 0.06 * max(xmax - xmin, ymax - ymin)

    # ---------------------------------------------------------
    # Plot inflated polygons (filled)
    # ---------------------------------------------------------
    for i, pid in enumerate(polygon_ids):
        ring = polygons_ccw_eps[pid]
        px, py = zip(*ring)
        col = polygon_colors[i]

        ax.fill(px, py, color=col, alpha=0.30)
        ax.plot(list(px) + [px[0]],
                list(py) + [py[0]],
                color=col,
                linewidth=1.4)

    # ---------------------------------------------------------
    # Plot original polygons (dashed outlines)
    # ---------------------------------------------------------
    for i, pid in enumerate(polygon_ids):
        if pid not in polygons_ccw_raw:
            continue

        ring_raw = polygons_ccw_raw[pid]
        ox, oy = zip(*ring_raw)
        col = polygon_colors[i]

        ax.plot(list(ox) + [ox[0]],
                list(oy) + [oy[0]],
                color=col,
                linewidth=1.0,
                linestyle="--",
                alpha=0.9)

    # ---------------------------------------------------------
    # Visibility edges (optional)
    # ---------------------------------------------------------
    if show_neighbors:
        for i, nbr_list in enumerate(neighbors):
            x1, y1 = node_coords[i]
            for j, dist_ij in nbr_list:
                x2, y2 = node_coords[j]
                ax.plot([x1, x2], [y1, y2],
                        color="black",
                        linewidth=0.6,
                        alpha=0.6)

    # ---------------------------------------------------------
    # A* optimal path (thick red)
    # ---------------------------------------------------------
    path_xy = [node_coords[i] for i in path_indices]
    px_path = [p[0] for p in path_xy]
    py_path = [p[1] for p in path_xy]
    
    path_line, = ax.plot(px_path, py_path,
                         color="red",
                         linewidth=2.5,
                         alpha=0.9,
                         zorder=10,
                         label="A* optimal path")

    # ---------------------------------------------------------
    # Odometry trajectory (empty initially, updated live)
    # ---------------------------------------------------------
    odom_line, = ax.plot([], [],
                         color="blue",
                         linewidth=1.8,
                         alpha=0.9,
                         zorder=12,
                         label="Odometry trajectory")
    
    # ---------------------------------------------------------
    # Camera trajectory (empty initially, updated live)
    # ---------------------------------------------------------
    cam_line, = ax.plot([], [],
                        color="green",
                        linewidth=1.8,
                        alpha=0.9,
                        zorder=13,
                        label="Camera trajectory")

    # ---------------------------------------------------------
    # Odometry arrow (current robot pose & heading)
    # ---------------------------------------------------------
    # Use the first point of the global path as initial pose
    x0, y0 = global_path[0]
    theta0 = initial_theta

    x_head = x0 + ARROW_LENGTH * np.cos(theta0)
    y_head = y0 + ARROW_LENGTH * np.sin(theta0)

    odom_arrow = FancyArrowPatch(
        posA=(x0, y0),
        posB=(x_head, y_head),
        arrowstyle="->",
        mutation_scale=15,   # arrowhead size
        color="blue",
        linewidth=2.0,
        zorder=20,
    )
    ax.add_patch(odom_arrow)

    # Small circle for the odometry robot base (so arrow is clearly attached)
    odom_circle = Circle((x0, y0), radius=ROBOT_RADIUS, facecolor="none",
                         edgecolor="blue", linewidth=1.8, zorder=19)
    ax.add_patch(odom_circle)

    # ---------------------------------------------------------
    # Camera arrow (camera pose & heading)
    # ---------------------------------------------------------
    cam_arrow = FancyArrowPatch(
        posA=(x0, y0),
        posB=(x0, y0),  # start with zero length; will be updated from vision
        arrowstyle="->",
        mutation_scale=15,
        color="green",
        linewidth=2.0,
        zorder=21,
    )
    ax.add_patch(cam_arrow)

    # Small circle for the camera robot base (on-map camera measurement)
    cam_circle = Circle((x0, y0), radius=ROBOT_RADIUS, facecolor="none",
                        edgecolor="green", linewidth=1.8, zorder=19)
    ax.add_patch(cam_circle)


    # ---------------------------------------------------------
    # Start and goal
    # ---------------------------------------------------------
    sx, sy = special_points["start"]
    gx, gy = special_points["goal"]

    ax.scatter([sx], [sy], s=80, color=COLOR_START, marker="o", zorder=15)
    ax.text(sx, sy, "  start", va="bottom", ha="left", fontsize=10)

    ax.scatter([gx], [gy], s=120, color=COLOR_GOAL, marker="*", zorder=15)
    ax.text(gx, gy, "  goal", va="bottom", ha="left", fontsize=10)

    # ---------------------------------------------------------
    # Axes cosmetics
    # ---------------------------------------------------------
    ax.set_xticks(np.arange(xmin, xmax + 1, TICK_STEP))
    ax.set_yticks(np.arange(ymin, ymax + 1, TICK_STEP))
    ax.grid(True, linewidth=0.5, alpha=0.3)

    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_title("Original vs Inflated Obstacles + A* Path + Odometry")

    # ---------------------------------------------------------
    # Legend outside the plot (on the right)
    # ---------------------------------------------------------
    legend_elements = []

    # World boundary (assume first polygon is poly0)
    world_color = polygon_colors[0]
    legend_elements.append(
        Patch(facecolor=world_color, edgecolor="none", alpha=0.25,
              label="World map (poly0)")
    )

    if num_polygons > 1:
        legend_elements.append(
            Patch(facecolor=polygon_colors[1], edgecolor=polygon_colors[1],
                  alpha=0.25, label="Inflated obstacles")
        )
        legend_elements.append(
            Line2D([], [], linestyle="--", color=polygon_colors[1],
                   label="Original obstacles")
        )

    # Start / goal
    legend_elements.extend([
        Line2D([], [], marker="o", linestyle="None",
               markerfacecolor=COLOR_START, markeredgecolor="none",
               label="Start"),
        Line2D([], [], marker="*", linestyle="None",
               markerfacecolor=COLOR_GOAL, markeredgecolor="none",
               label="Goal"),
    ])

    # Paths / edges
    legend_elements.append(
        Line2D([], [], color="red", linewidth=2.5, label="A* optimal path")
    )
    if show_neighbors:
        legend_elements.append(
            Line2D([], [], color="black", linewidth=0.6, label="Visibility edges")
        )
    legend_elements.append(
        Line2D([], [], color="blue", linewidth=1.8, label="Odometry")
    )
    legend_elements.append(
        Line2D([], [], color="green", linewidth=1.8, label="Camera")
    )

    # Epsilon info (text-only entry)
    legend_elements.append(
        Line2D([], [], color="none", label=f"ε = {EPSILON:.0f} mm")
    )

    ax.legend(
        handles=legend_elements,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.10),   # place BELOW plot
        ncol=3,                        # fit wide horizontally
        frameon=True
    )

    # Make room for legend
    plt.subplots_adjust(right=0.78)
    plt.tight_layout()

    plt.ion()
    fig.show()

    return (
        global_path,
        debug,
        fig,
        ax,
        odom_line,
        odom_arrow,
        odom_circle,
        cam_line,
        cam_arrow,
        cam_circle,
    )