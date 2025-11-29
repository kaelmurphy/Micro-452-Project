"""
dashboard.py

Standalone visualization dashboard with 3 panels:

- Left:   simulated camera feed (fake frames)
- Middle: map + inflated obstacles + A* path (from CSV, via setup_globalnav_plot)
- Right:  placeholder for Kalman covariance (empty for now)

This file is self-contained and does NOT require Thymio or a real camera.
"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle, FancyArrowPatch
import cv2  # used only to draw a simple moving blob

from globalnav_plot import setup_globalnav_plot, ARROW_LENGTH
from Kalman import ekf_step, wrap_angle  # uses your EKF implementation


# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------

CSV_PATH = "globalnav/CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv"

BOARD_WIDTH_MM  = 1255.0   # your paper width in mm
BOARD_HEIGHT_MM = 740.0    # your paper height in mm

# ----------------------------------------------------------------------
# DASHBOARD SETUP
# ----------------------------------------------------------------------

def setup_dashboard(
    coords: np.ndarray,
    initial_theta: float = 0.0,
    epsilon_mm: float = None,
    H_inv: np.ndarray = None, 
):
    """
    ...
    epsilon_mm : float
        Obstacle inflation radius in mm, passed through to setup_globalnav_plot.
        MUST be provided by the caller (e.g. GLOB_NAV_EPSILON from demo.py).
    """
    if epsilon_mm is None:
        raise ValueError("setup_dashboard: epsilon_mm must be provided (e.g. GLOB_NAV_EPSILON).")
    if H_inv is None:
        raise ValueError("setup_dashboard: H_inv must be provided (inverse homography world->pixel).")

    # --- Create figure and 3 panels using GridSpec ---
    fig = plt.figure(figsize=(19, 6))
    gs = GridSpec(
        1, 3,
        width_ratios=[3, 4, 2],   # wider left pane
        figure=fig
    )

    ax_cam = fig.add_subplot(gs[0, 0])
    ax_map = fig.add_subplot(gs[0, 1])

    # --- split right column into two rows: top=cov, bottom=error ---
    right_gs = gs[0, 2].subgridspec(2, 1, height_ratios=[3, 2], hspace=0.35)
    ax_cov = fig.add_subplot(right_gs[0, 0])   # top
    ax_err = fig.add_subplot(right_gs[1, 0])   # bottom

    # --- camera resolution (fake) ---
    cam_height = 720
    cam_width = 960

    # ------------------------------------------------------------------
    # LEFT PANEL: SIMULATED CAMERA
    # ------------------------------------------------------------------
    # Start with a black frame; we'll overwrite it each loop.
    fake_frame = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
    img_artist = ax_cam.imshow(fake_frame)
    ax_cam.set_title("Camera (simulated)")
    ax_cam.axis("off")

    # Make sure axes are in pixel coordinates:
    ax_cam.set_xlim(0, cam_width)
    ax_cam.set_ylim(cam_height, 0)  # y downwards like image

    # ------------------------------------------------------------------
    # MIDDLE PANEL: MAP / GLOBAL NAV
    # ------------------------------------------------------------------
    # We pass fig and ax so that setup_globalnav_plot draws into ax_map
    # instead of making its own figure. It should return handles
    # we can update later in a real control loop.
    #
    # NOTE: if the layout looks weird, you may want to guard the
    # internal plt.subplots_adjust / plt.tight_layout calls in
    # setup_globalnav_plot so they only run when it creates the figure.
    global_path, debug, fig_map, ax_map, \
    odom_line, odom_arrow, odom_circle_map, \
    cam_line, cam_arrow, cam_circle_map = setup_globalnav_plot(
        coords,
        epsilon_mm=epsilon_mm,
        show_neighbors=False,
        initial_theta=initial_theta,
        fig=fig,
        ax=ax_map,
    )
    
    # They should be the same figure
    # (this is more of a sanity check than a hard requirement)
    # fig_map is the same object as fig
    # print(fig is fig_map)  # can be used for debugging if needed

    # ------------------------------------------------------------------
    # PATH OVERLAY ON CAMERA (LEFT PANEL)
    # ------------------------------------------------------------------
    def world_to_pix(xw, yw):
        """
        Map world coordinates [mm] to pixel coordinates using H_inv.

        H maps pixel -> world.
        H_inv maps world -> pixel.
        """
        pts_world = np.array([[[xw, yw]]], dtype=np.float32)  # shape (1,1,2)
        pts_pix = cv2.perspectiveTransform(pts_world, H_inv)  # shape (1,1,2)
        px, py = pts_pix[0, 0]
        return float(px), float(py)

    # ------------------------------------------------------------------
    # CROP / ZOOM CAMERA VIEW TO BOARD USING H_inv (WITH ZOOM-OUT)
    # ------------------------------------------------------------------
    # World corners of the board in mm
    board_world = np.array(
        [
            [0.0,             0.0],
            [BOARD_WIDTH_MM,  0.0],
            [BOARD_WIDTH_MM,  BOARD_HEIGHT_MM],
            [0.0,             BOARD_HEIGHT_MM],
        ],
        dtype=np.float32
    ).reshape(-1, 1, 2)

    # Map those world corners to pixel coordinates with H_inv
    board_pix = cv2.perspectiveTransform(board_world, H_inv).reshape(-1, 2)

    x_min = board_pix[:, 0].min()
    x_max = board_pix[:, 0].max()
    y_min = board_pix[:, 1].min()
    y_max = board_pix[:, 1].max()

    # Compute center + size
    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    w  = (x_max - x_min)
    h  = (y_max - y_min)

    # Zoom *out* by this factor (>1.0 = show more around the board)
    ZOOM_OUT = 1.4   # try 1.2–1.6; increase if still too zoomed

    half_w = 0.5 * w * ZOOM_OUT
    half_h = 0.5 * h * ZOOM_OUT

    x_min_pix = cx - half_w
    x_max_pix = cx + half_w
    y_min_pix = cy - half_h
    y_max_pix = cy + half_h

    # Clamp to the actual image size [0, cam_width] x [0, cam_height]
    x_min_pix = max(0, x_min_pix)
    y_min_pix = max(0, y_min_pix)
    x_max_pix = min(cam_width,  x_max_pix)
    y_max_pix = min(cam_height, y_max_pix)

    # Apply limits (note: y inverted because image origin is top-left)
    ax_cam.set_xlim(x_min_pix, x_max_pix)
    ax_cam.set_ylim(y_max_pix, y_min_pix)


    # Map all path points to pixel coords
    px_list = []
    py_list = []
    for xw, yw in global_path:
        px, py = world_to_pix(xw, yw)
        px_list.append(px)
        py_list.append(py)

    # Colored dots for nodes (e.g. cyan) and red line for A* path
    path_nodes_scatter = ax_cam.scatter(
        px_list, py_list,
        s=25,
        c="cyan",
        zorder=5,
        label="Path nodes"
    )
    path_line_cam, = ax_cam.plot(
        px_list, py_list,
        color="red",
        linewidth=2,
        zorder=6,
        label="A* path"
    )

    # LEGEND UNDER THE LIVE IMAGE
    ax_cam.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.18),   # moved further down
        ncol=2,
        fontsize=11,                   # bigger font
        frameon=True
    )

    # ------------------------------------------------------------------
    # Camera-overlay: robot markers that sit on top of the left image
    # We'll draw a small circle to represent the robot base and an arrow
    # for heading for both odometry (blue) and camera (green).
    # Choose reasonable pixel sizes for the demo.
    ARROW_PIX = 40.0
    CIRCLE_PIX_RADIUS = 8.0

    # Use the first waypoint as initial pose for the overlay
    if len(px_list) > 0:
        initial_px, initial_py = px_list[0], py_list[0]
    else:
        initial_px, initial_py = cam_width / 2.0, cam_height / 2.0

    # Odometry marker on camera overlay (blue)
    odom_circle_cam = Circle((initial_px, initial_py), radius=CIRCLE_PIX_RADIUS,
                             facecolor="none", edgecolor="blue",
                             linewidth=1.6, zorder=30)
    ax_cam.add_patch(odom_circle_cam)

    odom_arrow_cam = FancyArrowPatch(
        posA=(initial_px, initial_py),
        posB=(initial_px + ARROW_PIX, initial_py),
        arrowstyle="->",
        mutation_scale=10,
        color="blue",
        linewidth=1.6,
        zorder=31,
    )
    ax_cam.add_patch(odom_arrow_cam)

    # Camera marker on camera overlay (green)
    cam_circle_cam = Circle((initial_px, initial_py), radius=CIRCLE_PIX_RADIUS,
                            facecolor="none", edgecolor="green",
                            linewidth=1.6, zorder=30)
    ax_cam.add_patch(cam_circle_cam)

    cam_arrow_cam = FancyArrowPatch(
        posA=(initial_px, initial_py),
        posB=(initial_px + ARROW_PIX, initial_py),
        arrowstyle="->",
        mutation_scale=10,
        color="green",
        linewidth=1.6,
        zorder=31,
    )
    ax_cam.add_patch(cam_arrow_cam)

    # RIGHT TOP: KALMAN COVARIANCE
    # ------------------------------------------------------------------
    ax_cov.set_title("EKF covariance (diagonal)")
    ax_cov.set_xlabel("time [s]")
    ax_cov.set_ylabel(r"$\mathrm{variance}\;[\mathrm{mm}^2,\;\mathrm{rad}^2]$")

    line_sigx, = ax_cov.plot([], [], label="σx²")
    line_sigy, = ax_cov.plot([], [], label="σy²")
    line_sigtheta, = ax_cov.plot([], [], label="σθ²")

    ax_cov.legend(loc="upper right")

    # ------------------------------------------------------------------
    # RIGHT BOTTOM: POSITION ERROR (ODOM vs CAMERA)
    # ------------------------------------------------------------------
    ax_err.set_title("Odom vs camera position error")
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("‖pos_est - pos_cam‖ [mm]")

    err_line, = ax_err.plot([], [], label="position error")
    ax_err.legend(loc="upper right")

    fig.tight_layout()
    # Extra bottom margin to give space for legends below plots
    fig.subplots_adjust(bottom=0.20)


    return {
        "fig": fig,
        "ax_cam": ax_cam,
        "img_artist": img_artist,
        "ax_map": ax_map,
        "global_path": global_path,
        "odom_line": odom_line,
        "odom_arrow": odom_arrow,
        "cam_line": cam_line,
        "cam_arrow": cam_arrow,
        "ax_cov": ax_cov,
        "ax_err": ax_err,
        "path_nodes_scatter": path_nodes_scatter,
        "path_line_cam": path_line_cam,
        "cam_width": cam_width,
        "cam_height": cam_height,
        "world_to_pix": world_to_pix,
        "cov_lines": (line_sigx, line_sigy, line_sigtheta),
        "err_line": err_line,
        # map-side circles (from setup_globalnav_plot)
        "odom_circle_map": odom_circle_map,
        "cam_circle_map": cam_circle_map,
        # camera-overlay robot markers (pixel coords)
        "odom_circle_cam": odom_circle_cam,
        "cam_circle_cam": cam_circle_cam,
        "odom_arrow_cam": odom_arrow_cam,
        "cam_arrow_cam": cam_arrow_cam,
    }


# ----------------------------------------------------------------------
# SIMPLE SIMULATION HELPERS
# ----------------------------------------------------------------------

def make_fake_camera_frame(t: float, width: int = 960, height: int = 720):
    """
    Create a simple synthetic camera frame:
    - black background
    - a colored circle moving in a Lissajous-ish pattern over time.

    Args:
        t:      time in seconds (or any continuous parameter)
        width:  frame width in pixels
        height: frame height in pixels

    Returns:
        frame:  (H, W, 3) uint8 BGR image
    """
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # Center and radii for the motion
    cx = int(width / 2 + (width / 4) * np.cos(0.7 * t))
    cy = int(height / 2 + (height / 4) * np.sin(1.1 * t))
    radius = 30

    # BGR color (green-ish)
    color = (0, 200, 100)

    cv2.circle(frame, (cx, cy), radius, color, thickness=-1)

    # You can add more fun things (moving rectangle, text, etc.) if you want
    return frame


def simulate_robot_along_path(global_path, step_size=10.0):
    """
    Generator that walks a virtual robot along the global_path, returning
    successive (x, y, theta) positions.

    Args:
        global_path: Nx2 array of waypoints (A* output)
        step_size:   distance [mm] moved per step

    Yields:
        x, y, theta: robot pose in map coords
    """
    # Start at first point
    x = float(global_path[0, 0])
    y = float(global_path[0, 1])
    theta = 0.0
    idx = 0

    while True:
        if idx >= len(global_path) - 1:
            # stay at the final goal and keep yielding pose
            yield x, y, theta
            continue

        x_target = float(global_path[idx + 1, 0])
        y_target = float(global_path[idx + 1, 1])

        dx = x_target - x
        dy = y_target - y
        dist = np.hypot(dx, dy)

        if dist < step_size:
            # jump to the next waypoint
            x, y = x_target, y_target
            idx += 1
        else:
            # move step_size towards target
            x += step_size * dx / dist
            y += step_size * dy / dist

        # heading along current segment
        if dist > 1e-9:
            theta = np.arctan2(dy, dx)

        yield x, y, theta

def make_world_to_pixel_mapper(global_path, img_width, img_height, margin_ratio=0.05):
    """
    Build a simple linear mapper from world coords (mm) to image pixels,
    just for visualization in this demo.

    We map the bounding box of global_path into the image, leaving a margin.
    """
    xs = global_path[:, 0]
    ys = global_path[:, 1]

    xmin, xmax = xs.min(), xs.max()
    ymin, ymax = ys.min(), ys.max()

    # Add small padding in world space to avoid clamping
    dx = xmax - xmin
    dy = ymax - ymin
    if dx == 0:
        dx = 1.0
    if dy == 0:
        dy = 1.0

    margin_x = img_width * margin_ratio
    margin_y = img_height * margin_ratio

    def world_to_pix(xw, yw):
        # Normalize to [0,1]
        nx = (xw - xmin) / dx
        ny = (yw - ymin) / dy

        # Scale into image with margins
        px = margin_x + nx * (img_width  - 2 * margin_x)
        py = margin_y + ny * (img_height - 2 * margin_y)

        # IMPORTANT: image y axis goes downward, but our mapping
        # above assumes y grows upward, so flip:
        py = img_height - py

        return px, py

    return world_to_pix

def world_to_pix_with_H(points_world, H_inv):
    pts_world = np.asarray(points_world, dtype=np.float32).reshape(-1, 1, 2)
    pts_pix = cv2.perspectiveTransform(pts_world, H_inv).reshape(-1, 2)
    return pts_pix[:,0], pts_pix[:,1]


# ----------------------------------------------------------------------
# MAIN DEMO LOOP
# ----------------------------------------------------------------------
