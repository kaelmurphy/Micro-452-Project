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
import cv2  # used only to draw a simple moving blob

from globalnav_plot import setup_globalnav_plot, ARROW_LENGTH
from Kalman import ekf_step, wrap_angle  # uses your EKF implementation


# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------

CSV_PATH = "globalnav/CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv"


# ----------------------------------------------------------------------
# DASHBOARD SETUP
# ----------------------------------------------------------------------

def setup_dashboard(csv_path: str = CSV_PATH):
    """
    Create the main figure and 3 subplots, initialize:
      - simulated camera image on the left,
      - map (via setup_globalnav_plot) in the center,
      - empty covariance panel on the right.

    Returns a bunch of handles you can use in an update loop.
    """

    # --- Load map from CSV, like in demo.py ---
    df = pd.read_csv(csv_path, sep=";")
    coords = df[["type", "id", "label", "x", "y"]].to_numpy(dtype=object)

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
    global_path, debug, fig_map, ax_map, odom_line, odom_arrow, cam_line, cam_arrow = setup_globalnav_plot(
        coords,
        epsilon_mm=50.0,
        show_neighbors=False,
        initial_theta=0.0,
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
    # Build a simple world->pixel mapping just for this demo:
    world_to_pix = make_world_to_pixel_mapper(global_path, cam_width, cam_height)

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
    # RIGHT TOP: KALMAN COVARIANCE
    # ------------------------------------------------------------------
    ax_cov.set_title("Kalman covariance")
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
        "ax_err": ax_err,                             # NEW
        "path_nodes_scatter": path_nodes_scatter,
        "path_line_cam": path_line_cam,
        "cam_width": cam_width,
        "cam_height": cam_height,
        "world_to_pix": world_to_pix,
        "cov_lines": (line_sigx, line_sigy, line_sigtheta),
        "err_line": err_line,                         # NEW
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

def run_demo():
    """
    Standalone demo:
      - creates the dashboard,
      - animates fake camera frames on the left,
      - moves a fake robot along the global path in the center.
    """
    handles = setup_dashboard()

    fig         = handles["fig"]
    ax_cam      = handles["ax_cam"]
    img_artist  = handles["img_artist"]
    ax_map      = handles["ax_map"]
    global_path = handles["global_path"]
    odom_line   = handles["odom_line"]
    odom_arrow  = handles["odom_arrow"]
    cam_line    = handles["cam_line"]
    cam_arrow   = handles["cam_arrow"]
    ax_cov      = handles["ax_cov"]
    ax_err      = handles["ax_err"]          # NEW
    cam_width   = handles["cam_width"]
    cam_height  = handles["cam_height"]
    line_sigx, line_sigy, line_sigtheta = handles["cov_lines"]
    err_line    = handles["err_line"]        # NEW

# ------------------------------------------------------------------
    # ROBOT PATH + EKF SETUP
    # ------------------------------------------------------------------
    step_size = 10.0  # mm / iteration
    robot_traj = simulate_robot_along_path(global_path, step_size=step_size)

    # Ground truth initial pose
    x_true, y_true, theta_true = next(robot_traj)
    x_prev_true, y_prev_true, theta_prev_true = x_true, y_true, theta_true

    # EKF state (estimate) and covariance
    x_est = np.array([[x_true],
                      [y_true],
                      [theta_true]])  # (3,1)

    Q = np.diag([0.005, 0.005, 0.002])
    R_cam = np.diag([0.002, 0.002, 0.0005])
    P = np.diag([1.0, 1.0, 0.5])

    # History for plotting covariance
    t_hist = []
    sigx_hist = []
    sigy_hist = []
    sigtheta_hist = []
    err_hist = []    # NEW: position error history

    # Initialize odometry trail with the first estimate
    xs = [float(x_est[0, 0])]
    ys = [float(x_est[1, 0])]
    odom_line.set_data(xs, ys)

    # Initial odometry arrow
    x0, y0 = xs[0], ys[0]
    theta0 = float(x_est[2, 0])
    x_head0 = x0 + ARROW_LENGTH * np.cos(theta0)
    y_head0 = y0 + ARROW_LENGTH * np.sin(theta0)
    odom_arrow.set_positions((x0, y0), (x_head0, y_head0))

    plt.ion()
    fig.show()

    t_start = time.time()
    t_last = t_start

    try:
        while True:
            t_now = time.time()
            t_rel = t_now - t_start
            Ts = t_now - t_last
            t_last = t_now
            if Ts <= 0:
                Ts = 0.02

            # ----------------------------------------------------------
            # 1) Update simulated camera frame (LEFT)
            # ----------------------------------------------------------
            frame_bgr = make_fake_camera_frame(t_rel, width=cam_width, height=cam_height)
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            img_artist.set_data(frame_rgb)

            # ----------------------------------------------------------
            # 2) Ground truth robot move along path
            # ----------------------------------------------------------
            x_true, y_true, theta_true = next(robot_traj)

            dx_true = x_true - x_prev_true
            dy_true = y_true - y_prev_true
            dist_true = np.hypot(dx_true, dy_true)
            v_wheel = dist_true / Ts

            dtheta_true = wrap_angle(theta_true - theta_prev_true)
            omega_wheel = dtheta_true / Ts

            x_prev_true, y_prev_true, theta_prev_true = x_true, y_true, theta_true

            # ----------------------------------------------------------
            # 3) Simulated camera measurement (noisy)
            # ----------------------------------------------------------
            z_cam = np.array([
                x_true  + np.random.normal(0.0, np.sqrt(R_cam[0, 0])),
                y_true  + np.random.normal(0.0, np.sqrt(R_cam[1, 1])),
                theta_true + np.random.normal(0.0, np.sqrt(R_cam[2, 2])),
            ])

            # ----------------------------------------------------------
            # 4) EKF STEP
            # ----------------------------------------------------------
            x_est, P = ekf_step(
                x_prev=x_est,
                P_prev=P,
                v_wheel=v_wheel,
                omega_wheel=omega_wheel,
                z_cam=z_cam,
                camera_on=True,
                Ts=Ts,
                Q=Q,
                R_cam=R_cam,
            )

            # ----------------------------------------------------------
            # 5) UPDATE MAP ODOMETRY WITH ESTIMATE (MIDDLE)
            # ----------------------------------------------------------
            x_est_val = float(x_est[0, 0])
            y_est_val = float(x_est[1, 0])
            theta_est = float(x_est[2, 0])

            xs.append(x_est_val)
            ys.append(y_est_val)
            odom_line.set_data(xs, ys)

            x_head = x_est_val + ARROW_LENGTH * np.cos(theta_est)
            y_head = y_est_val + ARROW_LENGTH * np.sin(theta_est)
            odom_arrow.set_positions((x_est_val, y_est_val), (x_head, y_head))

            # (We leave cam_line / cam_arrow unused here, but you have them.)

            # ----------------------------------------------------------
            # 6A) UPDATE COVARIANCE PLOT (RIGHT, top)
            # ----------------------------------------------------------
            t_hist.append(t_rel)
            sigx_hist.append(P[0, 0])
            sigy_hist.append(P[1, 1])
            sigtheta_hist.append(P[2, 2])

            line_sigx.set_data(t_hist, sigx_hist)
            line_sigy.set_data(t_hist, sigy_hist)
            line_sigtheta.set_data(t_hist, sigtheta_hist)

            ax_cov.relim()
            ax_cov.autoscale_view()

            # ----------------------------------------------------------
            # 6B) UPDATE ERROR PLOT (ODOM vs CAMERA, bottom right)
            # ----------------------------------------------------------
            pos_est = np.array([x_est_val, y_est_val])
            pos_cam = np.array([z_cam[0], z_cam[1]])   # camera "real" position

            err_val = np.linalg.norm(pos_est - pos_cam)  # Euclidean distance in mm
            err_hist.append(err_val)

            err_line.set_data(t_hist, err_hist)
            ax_err.relim()
            ax_err.autoscale_view()

            # ----------------------------------------------------------
            # 7) REDRAW
            # ----------------------------------------------------------
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Exiting dashboard demo.")


# ----------------------------------------------------------------------
# ENTRY POINT
# ----------------------------------------------------------------------

if __name__ == "__main__":
    run_demo()