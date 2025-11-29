import numpy as np
from matplotlib import pyplot as plt
from enum import Enum
from localnav import *
from globalnav import *
from Kalman import *
from globalnav_plot import setup_globalnav_plot, ARROW_LENGTH
from time import perf_counter
from vision2 import getVisionCoords, getRobotPositionMm
from threading import Thread

# Testing settings

NO_THYMIO_MODE = False
NO_CAMERA_MODE = False

# Thymio calibration settings
GLOB_NAV_EPSILON = 90
ERROR_TOLERANCE = 15

# Kalman settings

TS = 0.1
Q = np.diag([0.005, 0.005, 0.002])
R_cam = np.diag([0.002, 0.002, 0.0005])
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
                            path = path[1:]
                            thymio.set_target(path[1][0], path[1][1])
                        else:
                            state = State.END
                    elif thymio.is_blocked():
                        thymio.probe_obstacle(path)
                        state = State.FACE_AWAY

                case State.FACE_AWAY:
                    if thymio.is_clear():
                        THYMIO_LENGTH = 110
                        x = int(thymioPose[0] + THYMIO_LENGTH * np.cos(thymioPose[2]))
                        y = int(thymioPose[1] + THYMIO_LENGTH * np.sin(thymioPose[2]))
                        thymio.set_target(x, y)
                        state = State.EXIT_PATH

                case State.EXIT_PATH:
                    if np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < ERROR_TOLERANCE:
                        thymio.probe_obstacle(path)
                        state = State.PROBE_OBSTACLE

                case State.PROBE_OBSTACLE:
                    if thymio.is_clear():
                        THYMIO_LENGTH = 110
                        x = int(thymioPose[0] + THYMIO_LENGTH * np.cos(thymioPose[2]))
                        y = int(thymioPose[1] + THYMIO_LENGTH * np.sin(thymioPose[2]))
                        segment = np.array([[thymioPose[0], thymioPose[1]], [x, y]])
                        intersect, index = path_intersection_point(path, segment)
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
                        if np.linalg.norm([x - thymioPose[0], y - thymioPose[1]]) < ERROR_TOLERANCE:
                            thymio.probe_obstacle(path)
                            state = State.PROBE_OBSTACLE

            # Run Kalman filter

            #estimatedPose, P = ekf_step(thymioPose, P, v, omega, camPose, newCamPose, TS, Q, R_cam)
            newCamPose = False
            estimatedPose = camPose

            # Update position only if off by more than 30 mm (avoid unnecessary rollback)

            dx = np.abs(estimatedPose[0] - thymioPose[0])
            dy = np.abs(estimatedPose[1] - thymioPose[1])
            if dx > 15 or dy > 15:
                print(f"Correcting position by ({dx} mm, {dy} mm)")
                thymio.set_pose(estimatedPose[0], estimatedPose[1], estimatedPose[2])

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

    # Else capture first image

    else:
        coords, theta0, H = getVisionCoords(timeout=None, showDisplay=True)
        print(coords, "Initial orientation: {:.2f}".format(theta0))

    # Compute best path

    path = compute_global_path(coords, epsilon_mm=GLOB_NAV_EPSILON)
    print("A* global path:\n", path)

    # Global nav plot: map + polygons + A* path + empty odometry line

    global_path, debug, fig, ax, odom_line, odom_arrow, cam_line, cam_arrow = setup_globalnav_plot(
        coords,
        epsilon_mm=GLOB_NAV_EPSILON,
        show_neighbors=False,
        initial_theta=theta0,
    )

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

    while not closed:

        # Plot camera detected trajectory 

        xs_cam, ys_cam = cam_line.get_data()
        xs_cam = list(xs_cam) + [camPose[0]]
        ys_cam = list(ys_cam) + [camPose[1]]
        cam_line.set_data(xs_cam, ys_cam)

        # Plot camera detected heading 

        x_head_cam = camPose[0] + ARROW_LENGTH * np.cos(camPose[2])
        y_head_cam = camPose[1] + ARROW_LENGTH * np.sin(camPose[2])
        cam_arrow.set_positions((camPose[0], camPose[1]), (x_head_cam, y_head_cam))
        
        # Plot odometry detected trajectory 

        xs, ys = odom_line.get_data()
        xs = list(xs) + [thymio.x]
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

if __name__ == '__main__':

    try:
        main_thread()
    except KeyboardInterrupt:
        pass
