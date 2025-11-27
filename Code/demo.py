import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from enum import Enum
from tdmclient import aw
from localnav import *
from globalnav import *
from Kalman import *
from globalnav_plot import setup_globalnav_plot, ARROW_LENGTH
from vision import getVisionCoords
from time import perf_counter

def main():

    # Settings
    NO_THYMIO_MODE = False
    NO_CAMERA_MODE = False
    Q = np.diag([0.005, 0.005, 0.002])
    R_cam = np.diag([0.002, 0.002, 0.0005])
    P = np.diag([1.0, 1.0, 0.5])
    TARGET_TOLERANCE = 30

    # Capture first image
    if NO_CAMERA_MODE:
        df = pd.read_csv("globalnav/CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv", sep=";")
        coords = df[["type", "id", "label", "x", "y"]].to_numpy(dtype=object)
        theta0 = 0
    else:
        coords, theta0 = getVisionCoords(timeout=None, showDisplay=True)
        print(coords, "Initial orientation: {:.2f}".format(theta0))

    # Compute best path
    path = compute_global_path(coords, epsilon_mm=50.0)
    print("A* global path:\n", path)

    # Global nav plot: map + polygons + A* path + empty odometry line
    global_path, debug, fig, ax, odom_line, odom_arrow = setup_globalnav_plot(
        coords,
        epsilon_mm=50.0,
        show_neighbors=False,
        initial_theta=theta0,
    )

    # ---------------------------------------------------------
    # SIMULATION-ONLY MODE (NO THYMIO)
    # ---------------------------------------------------------
    if NO_THYMIO_MODE:
        print("SIMULATION MODE ENABLED - no Thymio, showing static map + arrow.")
        plt.ioff()
        plt.show()
        return


    # Local navigation state machine
    class State(Enum):
        FOLLOW          = 0
        FACE_AWAY       = 1
        EXIT_PATH       = 2
        PROBE_OBSTACLE  = 3
        NUDGE_FORWARD   = 4
        END             = 5

    # Connect Thymio
    with Thymio(THYMIO_482_CALIBRATION) as thymio:

        # Set initial position
        thymio.set_pose(path[0][0], path[0][1], theta0)

        # Initialize state machine
        thymio.set_target(path[1][0], path[1][1])
        state: State = State.FOLLOW
        lastTime = perf_counter()

        # Until path is completed
        try:
            # Until path is completed
            while state != State.END:

                # Local navigation
                match state:

                    case State.FOLLOW:
                        if np.linalg.norm([path[1][0] - thymio.x, path[1][1] - thymio.y]) < TARGET_TOLERANCE:
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
                            x = int(thymio.x + THYMIO_LENGTH * np.cos(thymio.theta))
                            y = int(thymio.y + THYMIO_LENGTH * np.sin(thymio.theta))
                            thymio.set_target(x, y)
                            state = State.EXIT_PATH

                    case State.EXIT_PATH:
                        if np.linalg.norm([x - thymio.x, y - thymio.y]) < TARGET_TOLERANCE:
                            thymio.probe_obstacle(path)
                            state = State.PROBE_OBSTACLE

                    case State.PROBE_OBSTACLE:
                        if thymio.is_clear():
                            THYMIO_LENGTH = 110
                            x = int(thymio.x + THYMIO_LENGTH * np.cos(thymio.theta))
                            y = int(thymio.y + THYMIO_LENGTH * np.sin(thymio.theta))
                            segment = np.array([[thymio.x, thymio.y], [x, y]])
                            intersect, index = path_intersection_point(path, segment)
                            if index is not None:
                                thymio.set_target(intersect[0], intersect[1])
                            else:
                                thymio.set_target(x, y)
                            state = State.NUDGE_FORWARD

                    case State.NUDGE_FORWARD:
                        if index is not None:
                            if np.linalg.norm([intersect[0] - thymio.x, intersect[1] - thymio.y]) < TARGET_TOLERANCE:
                                path = path[index:]
                                thymio.set_target(path[1][0], path[1][1])
                                state = State.FOLLOW
                        else:
                            if np.linalg.norm([x - thymio.x, y - thymio.y]) < TARGET_TOLERANCE:
                                thymio.probe_obstacle(path)
                                state = State.PROBE_OBSTACLE

                # TODO Camera acquisition
                coords, theta = getVisionCoords(timeout=None, showDisplay=False)

                # Run Kalman filter

                newTime = perf_counter()
                elapsed = newTime - lastTime
                lastTime = newTime

                z_cam = np.array([coords[4][3], coords[4][4], theta])
                x_prev = np.array([thymio.x, thymio.y, thymio.theta])
                x_new, P = ekf_step(x_prev, P, thymio.v, thymio.omega, z_cam, True, elapsed, Q, R_cam)

                # Update position with new estimation
                thymio.set_pose(x_new[0], x_new[1], x_new[2])
                
                # -------------------------------------------------
                # UPDATE MATPLOTLIB PLOT (ODOMETRY + ARROW)
                # -------------------------------------------------
                # 1) Update odometry trail
                xs, ys = odom_line.get_data()
                xs = list(xs) + [thymio.x]
                ys = list(ys) + [thymio.y]
                odom_line.set_data(xs, ys)

                # 2) Update odometry arrow
                x = thymio.x
                y = thymio.y
                theta = thymio.theta

                x_head = x + ARROW_LENGTH * np.cos(theta)
                y_head = y + ARROW_LENGTH * np.sin(theta)
                odom_arrow.set_positions((x, y), (x_head, y_head))

                fig.canvas.draw()
                fig.canvas.flush_events()

                # Pace loop at 20 ms
                aw(thymio.client.sleep(0.02))
                # print(f"Elapsed time: {elapsed * 1000:.1f} ms, time to go {(SAMPLE_PERIOD - elapsed) * 1000:.1f} ms")
                # if elapsed < SAMPLE_PERIOD:
                #     aw(thymio.client.sleep(SAMPLE_PERIOD - elapsed))
        
        finally:
            print("Exiting control loop.")

    plt.ioff()
    plt.show()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
