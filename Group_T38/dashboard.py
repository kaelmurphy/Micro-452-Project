"""
dashboard.py

Standalone visualization dashboard with 3 panels:

- Left:  simulated camera view with global A* path and robot pose overlay
- Middle: global navigation map (inflated obstacles, visibility-graph A* path,
          odometry and camera-based trajectories)
- Right: EKF covariance evolution and odometry vs EKF position error

This file is self-contained and does NOT require Thymio or a real camera.
It is intended as the main entry point to visualize the overall system
(global navigation + perception + filtering) with pre-recorded or simulated data.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle, FancyArrowPatch
import cv2  # used only to draw a simple moving blob

from globalnav_plot import setup_globalnav_plot, ARROW_LENGTH
#from Kalman import ekf_step, wrap_angle  # uses your EKF implementation


# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------
CSV_PATH = "globalnav/CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv"

BOARD_WIDTH_MM  = 1255.0   # paper width in mm (global map)
BOARD_HEIGHT_MM = 740.0    # paper height in mm

# Default camera resolution for simulation (pixels)
DEFAULT_CAM_WIDTH  = 640
DEFAULT_CAM_HEIGHT = 480

# Overlay appearance (pixels)
CAM_MARGIN_PIX       = 30    # margin around board when cropping camera view
ARROW_PIX            = 40.0  # arrow length for camera overlay
CIRCLE_PIX_RADIUS    = 8.0   # radius for robot markers on the camera image

# ----------------------------------------------------------------------
# DASHBOARD SETUP
# ----------------------------------------------------------------------

def setup_dashboard(
    coords: np.ndarray,
    initial_theta: float = 0.0,
    epsilon_mm: float = None,
    H_inv: np.ndarray = None,
    board_corners_pix: np.ndarray = None,
    cam_width = None,
    cam_height = None,
):
    """
    Create and initialize the visualization dashboard.

    The dashboard consists of:
    - a camera panel with A* path and robot pose overlays (left),
    - a global navigation map (middle),
    - EKF covariance and odometry vs EKF error plots (right).

    Parameters
    ----------
    coords : np.ndarray
        Map description passed to `setup_globalnav_plot`, typically an array
        or DataFrame-compatible structure with columns ["type", "id", "label", "x", "y"].
        This encodes polygons, start, and goal in world coordinates [mm].
    initial_theta : float, optional
        Initial robot heading [rad], used for the first odometry arrow.
    epsilon_mm : float
        Obstacle inflation radius in mm (configuration-space margin). This is
        passed through to `setup_globalnav_plot` and must match the value used
        in the global navigation experiments (see report, Global Navigation).
    H_inv : np.ndarray
        3x3 inverse homography mapping world coordinates [mm] to pixel coordinates.
        Used to project the global path and robot pose onto the camera image.
    board_corners_pix : np.ndarray, optional
        Array of shape (4, 2) with the board corners in pixel coordinates.
        If provided, the camera view is cropped around the board with a margin.
    cam_width : int, optional
        Width of the camera frame in pixels. If None, uses DEFAULT_CAM_WIDTH.
    cam_height : int, optional
        Height of the camera frame in pixels. If None, uses DEFAULT_CAM_HEIGHT.

    Returns
    -------
    handles : dict
        Dictionary of Matplotlib artists and helper functions used to update
        the dashboard in a control loop. Keys include:
        - "fig": main Figure
        - "ax_cam", "ax_map", "ax_cov", "ax_err": axes objects
        - "img_artist": camera image artist
        - "global_path": A* waypoint sequence
        - "odom_line", "odom_arrow", "odom_circle_map": odometry map artists
        - "cam_line", "cam_arrow", "cam_circle_map": camera map artists
        - "odom_circle_cam", "cam_circle_cam", "odom_arrow_cam", "cam_arrow_cam":
          camera overlay markers in pixel space
        - "cov_lines": EKF covariance line objects
        - "err_line": odom vs EKF position error line
        - "world_to_pix": helper function mapping world [mm] → pixel coords.
    """

    if epsilon_mm is None:
        raise ValueError("setup_dashboard: epsilon_mm must be provided (e.g. GLOB_NAV_EPSILON).")
    if H_inv is None:
        raise ValueError("setup_dashboard: H_inv must be provided (inverse homography world->pixel).")
    # Fallback defaults if not passed (ONLY used in simulation / tests)
    if cam_width is None or cam_height is None:
        cam_width = DEFAULT_CAM_WIDTH
        cam_height = DEFAULT_CAM_HEIGHT

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

    # ------------------------------------------------------------------
    # LEFT PANEL: SIMULATED CAMERA
    # ------------------------------------------------------------------
    # Start with a black frame; we'll overwrite it each loop.
    fake_frame = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
    img_artist = ax_cam.imshow(fake_frame)
    ax_cam.set_title("Camera")
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
    # CROP CAMERA VIEW AROUND THE FOUR MARKERS
    # ------------------------------------------------------------------
    if board_corners_pix is not None:
        # board_corners_pix is 4x2, type float32
        board_pix = np.asarray(board_corners_pix, dtype=np.float32).reshape(-1, 2)

        x_min_pix = board_pix[:, 0].min()
        x_max_pix = board_pix[:, 0].max()
        y_min_pix = board_pix[:, 1].min()
        y_max_pix = board_pix[:, 1].max()

        # Add margin so the markers & paper are fully visible
        margin = CAM_MARGIN_PIX
        x_min_pix = max(0, x_min_pix - margin)
        x_max_pix = min(cam_width,  x_max_pix + margin)
        y_min_pix = max(0, y_min_pix - margin)
        y_max_pix = min(cam_height, y_max_pix + margin)

        # Apply limits (note: y inverted for images)
        ax_cam.set_xlim(x_min_pix, x_max_pix)
        ax_cam.set_ylim(y_max_pix, y_min_pix)
    else:
        # Fallback: full frame
        ax_cam.set_xlim(0, cam_width)
        ax_cam.set_ylim(cam_height, 0)

    # ------------------------------------------------------------------
    # DEBUG: draw board corners from world coordinates (should align with ArUco markers).
    # ------------------------------------------------------------------
    board_world_mm = np.array([
        [0.0,             0.0],
        [BOARD_WIDTH_MM,  0.0],
        [BOARD_WIDTH_MM,  BOARD_HEIGHT_MM],
        [0.0,             BOARD_HEIGHT_MM],
    ])

    bx, by = [], []
    for xw, yw in board_world_mm:
        px, py = world_to_pix(xw, yw)
        bx.append(px)
        by.append(py)

    ax_cam.scatter(bx, by, s=40, c="yellow", marker="x", zorder=200)


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
    # Shows the diagonal of the EKF covariance matrix P (σx², σy², σθ²) over time.

    # Left y-axis:   σx², σy²    (mm²)
    # Right y-axis:  σθ²         (rad²)

    ax_cov.set_title("EKF covariance (diagonal)")
    ax_cov.set_xlabel("time [s]")

    # Create a twin axis on the right
    ax_cov_theta = ax_cov.twinx()
    
    # Enable autoscaling for θ-axis
    ax_cov_theta.set_autoscale_on(True)
    ax_cov_theta.relim()
    ax_cov_theta.autoscale_view()

    # Left-axis lines: σx², σy²
    line_sigx, = ax_cov.plot([], [], label=r"$\sigma_x^2$")
    line_sigy, = ax_cov.plot([], [], label=r"$\sigma_y^2$")

    ax_cov.set_ylabel(r"$\sigma_x^2,\,\sigma_y^2\;\;[\mathrm{mm}^2]$")
    ax_cov.grid(True, alpha=0.3)

    # Right-axis line: σθ²
    line_sigtheta, = ax_cov_theta.plot([], [], linestyle="--",
                                    color="tab:red",
                                    label=r"$\sigma_\theta^2$")

    ax_cov_theta.set_ylabel(r"$\sigma_\theta^2\;[\mathrm{rad}^2]$")

    # Combined legend (collect from both axes)
    lines = [line_sigx, line_sigy, line_sigtheta]
    labels = [l.get_label() for l in lines]
    ax_cov.legend(lines, labels, loc="upper right")


    # ------------------------------------------------------------------
    # RIGHT BOTTOM: POSITION ERROR (ODOM vs CAMERA)
    # ------------------------------------------------------------------
    ax_err.set_title("Odom vs EKF estimate")
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("‖pos_thymio - pos_est‖ [mm]")

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
