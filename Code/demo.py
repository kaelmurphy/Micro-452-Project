import numpy as np
from matplotlib import pyplot as plt
from enum import Enum
from localnav import *
from globalnav import *
from Kalman import *
from globalnav_plot import ARROW_LENGTH
from time import perf_counter
from vision2 import getVisionCoords, getRobotPositionMm, getLiveFrameBGR
from threading import Thread
from dashboard import setup_dashboard
import pandas as pd
import cv2

# Testing settings

NO_THYMIO_MODE = False
NO_CAMERA_MODE = False

# Thymio calibration settings
GLOB_NAV_EPSILON = 90
WAYPOINT_POS_TOLERANCE = 15
NUDGE_LENGTH = 110
SOFT_KIDNAPPING_THRESHOLD = 20
HARD_KIDNAPPING_THRESHOLD = 200

# Kalman settings

TS = 0.1
Q = np.diag([0.000001, 0.000001, 1.2e-2])
R_cam = np.diag([0.0005, 0.0005, 0.0005])
P = np.diag([1.0, 1.0, 0.5])

# Globals

thymio: Thymio = None
camPose: np.ndarray = np.zeros(3)
estPose: np.ndarray = np.zeros(3)
newCamPose: bool = None
robotSeen: bool = False
hardKidnapping: np.ndarray = None

# Threads

def thymio_thread(path: np.ndarray, theta0: float) -> None:

    # Shared globals

    global WAYPOINT_POS_TOLERANCE, NUDGE_LENGTH, SOFT_KIDNAPPING_THRESHOLD, HARD_KIDNAPPING_THRESHOLD, TS
    global Q, R_cam, P, thymio, camPose, estPose, newCamPose, hardKidnapping
    
    # Local navigation state machine

    class State(Enum):
        FOLLOW          = 0
        FACE_AWAY       = 1
        EXIT_PATH       = 2
        PROBE_OBSTACLE  = 3
        NUDGE_FORWARD   = 4
        END             = 5

    # Connect Thymio

    with Thymio(path[0][0], path[0][1], theta0, THYMIO_482_CALIBRATION) as thymio:

        # Initialize motor control and local navigation

        thymio.set_target(path[1][0], path[1][1])
        state: State = State.FOLLOW
        t0 = perf_counter()

        # Until path is completed

        while state != State.END:

            # Wait one sampling period

            t1 = perf_counter()
            dt = t1 - t0
            t0 = t1
            aw(thymio.client.sleep(max([0.001, TS - dt])))

            # Acquire sample
            
            thymioPose = np.array([thymio.x, thymio.y, thymio.theta])
            v = thymio.v
            omega = thymio.omega

            # Run local navigation (avoidance) state machine

            match state:

                case State.FOLLOW:
                    if np.linalg.norm([path[1][0] - thymioPose[0], path[1][1] - thymioPose[1]]) < WAYPOINT_POS_TOLERANCE:
                        if path.shape[0] > 2:
                            previousWaypoint = path[0]
                            path = path[1:]
                            thymio.set_target(path[1][0], path[1][1])
                        else:
                            state = State.END
                    elif thymio.is_blocked():
                        avoidancePath = path
                        avoidancePath[0] = thymioPose[:2]
                        if path.shape[0] == 2:
                            directionPath = np.insert(path, 0, previousWaypoint, axis=0)
                            thymio.set_avoidance_direction(trajectory_direction(directionPath))
                        else:
                            thymio.set_avoidance_direction(trajectory_direction(path))
                        thymio.probe_obstacle(path)
                        state = State.FACE_AWAY

                case State.FACE_AWAY:
                    if thymio.is_clear():
                        x = int(thymioPose[0] + NUDGE_LENGTH * np.cos(thymioPose[2]))
                        y = int(thymioPose[1] + NUDGE_LENGTH * np.sin(thymioPose[2]))
                        thymio.set_target(x, y)
                        state = State.EXIT_PATH

                case State.EXIT_PATH:
                    if thymio.is_blocked() or np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < WAYPOINT_POS_TOLERANCE:
                        thymio.probe_obstacle(path)
                        state = State.PROBE_OBSTACLE

                case State.PROBE_OBSTACLE:
                    if thymio.is_clear():
                        x = int(thymioPose[0] + NUDGE_LENGTH * np.cos(thymioPose[2]))
                        y = int(thymioPose[1] + NUDGE_LENGTH * np.sin(thymioPose[2]))
                        nextMoveLine = np.array([[thymioPose[0], thymioPose[1]], [x, y]])
                        intersect, index = path_intersection_point(avoidancePath, nextMoveLine)
                        if index is not None:
                            thymio.set_target(intersect[0], intersect[1])
                        else:
                            thymio.set_target(x, y)
                        state = State.NUDGE_FORWARD

                case State.NUDGE_FORWARD:
                    if index is not None:
                        if np.linalg.norm([intersect[0] - thymioPose[0], intersect[1] - thymioPose[1]]) < WAYPOINT_POS_TOLERANCE:
                            path = path[index:]
                            thymio.set_target(path[1][0], path[1][1])
                            state = State.FOLLOW
                    else:
                        if thymio.is_blocked() or np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < WAYPOINT_POS_TOLERANCE:
                            thymio.probe_obstacle(path)
                            state = State.PROBE_OBSTACLE

            # Run Kalman filter
            est, P = ekf_step(thymioPose, P, v, omega, camPose, newCamPose and robotSeen, TS, Q, R_cam)
            estPose = est.flatten()
            newCamPose = False

            # Early return if in avoidance mode
            if state != State.FOLLOW:
                return
            
            # Detect soft and hard kidnapping cases
            errorNorm = np.linalg.norm((np.round(estPose[:2]) - thymioPose[:2]))
            if errorNorm > HARD_KIDNAPPING_THRESHOLD:
                print(f"Hard kidnapping, recomputing path")
                hardKidnapping = camPose
                break
            elif errorNorm > SOFT_KIDNAPPING_THRESHOLD:
                print(f"Soft kidnapping, correcting position by ({errorNorm} mm)")
                thymio.set_pose(estPose[0], estPose[1], estPose[2])

        # Stop thymio

        thymio.node.stop()

def camera_thread() -> None:

    # Globals

    global camPose, newCamPose, robotSeen
    
    while True:

        x_mm, y_mm, theta, robotSeen = getRobotPositionMm(showDisplay=False)
        camPose = np.array([x_mm, y_mm, theta])
        newCamPose = True

def main_thread() -> None:

    # Globals

    global NO_CAMERA_MODE, NO_THYMIO_MODE, GLOB_NAV_EPSILON, thymio, camPose, estPose, newCamPose, robotSeen, hardKidnapping

    # Use simulation map if camera is not available

    if NO_CAMERA_MODE:
        df = pd.read_csv("simulation.csv", sep=";")
        coords = df[["type", "id", "label", "x", "y"]].to_numpy(dtype=object)
        theta0 = 0
        board_corners_pix = None
        cam_width = 640
        cam_height = 480
        H_inv = np.eye(3, dtype=float)  # dummy, only used for mapping

    # Else capture first image

    else:
        #coords, theta0, H = getVisionCoords(timeout=None, showDisplay=True)
        coords, theta0, H, board_corners_pix = getVisionCoords(timeout=None, showDisplay=True)
        print(coords, "Initial orientation: {:.2f}".format(theta0))
        #Calculate inverse homography for later use
        H_inv = np.linalg.inv(H)
        frame0 = getLiveFrameBGR()
        cam_height, cam_width = frame0.shape[:2]

    # Compute best path

    handles = setup_dashboard(
    coords=coords,
    initial_theta=theta0,
    epsilon_mm=GLOB_NAV_EPSILON,
    H_inv=H_inv,
    board_corners_pix=board_corners_pix,
    cam_width=cam_width,
    cam_height=cam_height,
    )

    fig             = handles["fig"]
    img_artist      = handles["img_artist"]
    global_path     = handles["global_path"]
    odom_line       = handles["odom_line"]
    odom_arrow      = handles["odom_arrow"]
    odom_circle_map = handles["odom_circle_map"]
    cam_line        = handles["cam_line"]
    cam_arrow       = handles["cam_arrow"]
    cam_circle_map  = handles["cam_circle_map"]

    ax_cov          = handles["ax_cov"]
    ax_err          = handles["ax_err"]
    line_sigx, line_sigy, line_sigtheta = handles["cov_lines"]
    err_line        = handles["err_line"]

    # camera-overlay & mapping handles
    world_to_pix    = handles["world_to_pix"]
    odom_circle_cam = handles["odom_circle_cam"]
    cam_circle_cam  = handles["cam_circle_cam"]
    odom_arrow_cam  = handles["odom_arrow_cam"]
    cam_arrow_cam   = handles["cam_arrow_cam"]

    path = global_path.copy()
    print("A* global path:\n", path)

    # Only plot global nav path if thymio is not available

    if NO_THYMIO_MODE:
        print("SIMULATION MODE ENABLED - no Thymio, showing static map + arrow.")
        plt.ioff()
        plt.show()
        return
    
    # Only start camera thread if camera is available
    if not NO_CAMERA_MODE:
        cameraThread = Thread(target=camera_thread, daemon=True)
        cameraThread.start()

    # Start thymio thread

    thymioThread = Thread(target=thymio_thread, args=(path, theta0), daemon=True)
    thymioThread.start()
    while thymio is None:
        plt.pause(0.001)

    # Run until window is closed

    closed = False

    #histories for right-hand plots
    t0 = perf_counter()
    t_hist = []
    sigx_hist = []
    sigy_hist = []
    sigtheta_hist = []
    err_hist = []


    def on_close(event):
        nonlocal closed
        closed = True

    fig.canvas.mpl_connect('close_event', on_close)

    log = 'time (s), cam_x (mm), cam_y (mm), cam_theta (rad), odom_x (mm), odom_y (mm), odom_theta (rad), v (mm/s), omega (rad/s), est_x (mm), est_y (mm), est_theta (rad)\n'

    while not closed:

        # Handle hard kidnapping by restarting thymio thread

        if hardKidnapping is not None:

            coords[4][3] = hardKidnapping[0]
            coords[4][4] = hardKidnapping[1]
            theta0 = hardKidnapping[2]

            print("Recomputing global path...")

            hardKidnapping = None

            while thymioThread.is_alive():
                plt.pause(0.1)

            handles = setup_dashboard(
                coords=coords,
                initial_theta=theta0,
                epsilon_mm=GLOB_NAV_EPSILON,
                H_inv=H_inv,
                board_corners_pix=board_corners_pix,
                cam_width=cam_width,
                cam_height=cam_height,
            )

            fig             = handles["fig"]
            img_artist      = handles["img_artist"]
            global_path     = handles["global_path"]
            odom_line       = handles["odom_line"]
            odom_arrow      = handles["odom_arrow"]
            odom_circle_map = handles["odom_circle_map"]
            cam_line        = handles["cam_line"]
            cam_arrow       = handles["cam_arrow"]
            cam_circle_map  = handles["cam_circle_map"]

            ax_cov          = handles["ax_cov"]
            ax_err          = handles["ax_err"]
            line_sigx, line_sigy, line_sigtheta = handles["cov_lines"]
            err_line        = handles["err_line"]

            #camera-overlay & mapping handles
            world_to_pix    = handles["world_to_pix"]
            odom_circle_cam = handles["odom_circle_cam"]
            cam_circle_cam  = handles["cam_circle_cam"]
            odom_arrow_cam  = handles["odom_arrow_cam"]
            cam_arrow_cam   = handles["cam_arrow_cam"]

            path = global_path.copy()

            thymioThread = Thread(target=thymio_thread, args=(path, theta0), daemon=True)
            thymioThread.start()
            while thymio is None:
                plt.pause(0.001)

        # --- 1) Update live camera image on the left panel ---
        try:
            frame_bgr = getLiveFrameBGR()
            if frame_bgr is not None:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                img_artist.set_data(frame_rgb)
        except RuntimeError:
            # Vision not initialized or camera problem -> just skip
            pass

        if robotSeen and newCamPose:

            # Plot camera detected trajectory 

            xs_cam, ys_cam = cam_line.get_data()
            xs_cam = list(xs_cam) + [camPose[0]]
            ys_cam = list(ys_cam) + [camPose[1]]
            cam_line.set_data(xs_cam, ys_cam)

            # Plot camera detected heading 

            x_head_cam = camPose[0] + ARROW_LENGTH * np.cos(camPose[2])
            y_head_cam = camPose[1] + ARROW_LENGTH * np.sin(camPose[2])
            cam_arrow.set_positions((camPose[0], camPose[1]), (x_head_cam, y_head_cam))
            try:
                cam_circle_map.center = (camPose[0], camPose[1])
            except Exception:
                pass
            
        # Plot odometry detected trajectory 

        xs, ys = odom_line.get_data()
        xs = list(xs) + [thymio.x]
        try:
            odom_circle_map.center = (thymio.x, thymio.y)
        except Exception:
            pass
        ys = list(ys) + [thymio.y]
        odom_line.set_data(xs, ys)

        # Plot odometry detected heading 

        x = thymio.x
        y = thymio.y
        theta_est = thymio.theta
        x_head = x + ARROW_LENGTH * np.cos(theta_est)
        y_head = y + ARROW_LENGTH * np.sin(theta_est)
        odom_arrow.set_positions((x, y), (x_head, y_head))

        # ----------------------------------------------------------
        #  UPDATE CAMERA-OVERLAY MARKERS ON LEFT PANEL
        # ----------------------------------------------------------
        ARROW_PIX = 40.0

        # 1) ODOM overlaid on camera image
        px_odom, py_odom = world_to_pix(thymio.x, thymio.y)
        odom_circle_cam.center = (px_odom, py_odom)

        px_head_odom = px_odom + ARROW_PIX * np.cos(thymio.theta)
        py_head_odom = py_odom - ARROW_PIX * np.sin(thymio.theta)   # minus: screen y grows downward
        odom_arrow_cam.set_positions((px_odom, py_odom),
                                    (px_head_odom, py_head_odom))

        # 2) CAMERA measurement overlaid on camera image
        if robotSeen:
            px_cam, py_cam = world_to_pix(camPose[0], camPose[1])
            cam_circle_cam.center = (px_cam, py_cam)

            px_head_cam = px_cam + ARROW_PIX * np.cos(camPose[2])
            py_head_cam = py_cam - ARROW_PIX * np.sin(camPose[2])
            cam_arrow_cam.set_positions((px_cam, py_cam),
                                        (px_head_cam, py_head_cam))

        # ---------- UPDATE COVARIANCE PLOT (top-right) ----------
        t_rel = perf_counter() - t0
        t_hist.append(t_rel)

        # P is global and updated inside thymio_thread by ekf_step(...)
        # We assume P is a 3x3 numpy array.
        sigx_hist.append(P[0, 0])
        sigy_hist.append(P[1, 1])
        sigtheta_hist.append(P[2, 2])

        line_sigx.set_data(t_hist, sigx_hist)
        line_sigy.set_data(t_hist, sigy_hist)
        line_sigtheta.set_data(t_hist, sigtheta_hist)
        ax_cov.relim()
        ax_cov.autoscale_view()

        # ---------- UPDATE ERROR PLOT (odom vs camera, bottom-right) ----------

        pos_odom = np.array([thymio.x, thymio.y])
        pos_cam  = np.array([estPose[0], estPose[1]])
        err_val = np.linalg.norm(pos_odom - pos_cam)   # [mm]

        # Clamp error value for better visualization
        if err_val > 2 * WAYPOINT_POS_TOLERANCE:
            err_val = 2 * WAYPOINT_POS_TOLERANCE

        err_hist.append(err_val)

        err_line.set_data(t_hist, err_hist)
        ax_err.relim()
        ax_err.autoscale_view()

        # Update figure

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.025)

        log += f'{thymio.t}, {camPose[0]}, {camPose[1]}, {camPose[2]}, {thymio.x}, {thymio.y}, {thymio.theta}, {thymio.v}, {thymio.omega}\n'

    with open('log.csv', 'w') as log_file:
        log_file.write(log)

if __name__ == '__main__':

    try:
        main_thread()
    except KeyboardInterrupt:
        pass
