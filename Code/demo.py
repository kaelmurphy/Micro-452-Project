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


# Testing settings

NO_THYMIO_MODE = False
NO_CAMERA_MODE = False

# Thymio calibration settings
GLOB_NAV_EPSILON = 90
ERROR_TOLERANCE = 15
NUDGE_LENGTH = 110
SOFT_KIDNAPPING_THRESHOLD = 15

# Kalman settings

TS = 0.1
Q = np.diag([0.000001, 0.000001, 1.2e-2])
R_cam = np.diag([0.0005, 0.0005, 0.0005])
P = np.diag([1.0, 1.0, 0.5])

# Globals

thymio: Thymio = None
camPose: np.ndarray = np.zeros(3)
newCamPose: bool = None

# Threads

def thymio_thread(path: np.ndarray, theta0: float) -> None:

    # Shared globals

    global ERROR_TOLERANCE, TS, Q, R_cam, P, thymio, camPose, newCamPose
    
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
                    if np.linalg.norm([path[1][0] - thymioPose[0], path[1][1] - thymioPose[1]]) < ERROR_TOLERANCE:
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
                    if np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < ERROR_TOLERANCE:
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
                        if np.linalg.norm([intersect[0] - thymioPose[0], intersect[1] - thymioPose[1]]) < ERROR_TOLERANCE:
                            path = path[index:]
                            thymio.set_target(path[1][0], path[1][1])
                            state = State.FOLLOW
                    else:
                        if thymio.is_blocked() or np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < ERROR_TOLERANCE:
                            thymio.probe_obstacle(path)
                            state = State.PROBE_OBSTACLE

            # Run Kalman filter

            estimatedPose, P = ekf_step(thymioPose, P, v, omega, camPose, newCamPose, TS, Q, R_cam)
            newCamPose = False

            # Update position only if off by more than 30 mm (avoid unnecessary rollback)

            errorNorm = np.linalg.norm((np.round(estimatedPose[:2, 0]) - thymioPose[:2]))
            if errorNorm > SOFT_KIDNAPPING_THRESHOLD:
                print(f"Correcting position by ({errorNorm} mm)")
                thymio.set_pose(estimatedPose[0][0], estimatedPose[1][0], estimatedPose[2][0])

        # Stop thymio

        thymio.node.stop()

def camera_thread() -> None:

    # Globals

    global camPose, newCamPose
    
    while True:

        x_mm, y_mm, theta = getRobotPositionMm(showDisplay=False)
        camPose = np.array([x_mm, y_mm, theta])
        newCamPose = True

def main_thread() -> None:

    # Globals

    global NO_CAMERA_MODE, NO_THYMIO_MODE, GLOB_NAV_EPSILON, thymio, camPose

    # Use simulation map if camera is not available

    if NO_CAMERA_MODE:
        df = pd.read_csv("simulation.csv", sep=";")
        coords = df[["type", "id", "label", "x", "y"]].to_numpy(dtype=object)
        theta0 = 0
        H_inv = np.eye(3, dtype=float)  # dummy, only used for mapping

    # Else capture first image

    else:
        coords, theta0, H = getVisionCoords(timeout=None, showDisplay=True)
        print(coords, "Initial orientation: {:.2f}".format(theta0))
        #Calculate inverse homography for later use
        H_inv = np.linalg.inv(H)
    # Compute best path

    handles = setup_dashboard(
    coords=coords,
    initial_theta=theta0,
    epsilon_mm=GLOB_NAV_EPSILON,
    H_inv=H_inv,
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

    def on_close(event):
        nonlocal closed
        closed = True

    fig.canvas.mpl_connect('close_event', on_close)

    log = 'time (s), cam_x (mm), cam_y (mm), cam_theta (rad), odom_x (mm), odom_y (mm), odom_theta (rad), v (mm/s), omega (rad/s), est_x (mm), est_y (mm), est_theta (rad)\n'

    while not closed:

        # --- 1) Update live camera image on the left panel ---
        try:
            frame_bgr = getLiveFrameBGR()
            if frame_bgr is not None:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                img_artist.set_data(frame_rgb)
        except RuntimeError:
            # Vision not initialized or camera problem -> just skip
            pass

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
