import numpy as np
from matplotlib import pyplot as plt
from enum import Enum
from tdmclient import aw
from localnav import *
from globalnav import *

def main():

    # TODO Acquire first image
    theta0 = 0

    # Find optimal path
    df = pd.read_csv("globalnav/CSV_Data/Simulation_Data_VG_V1.0_13.11.25.csv", sep=";")
    map_array = df[["type", "id", "label", "x", "y"]].to_numpy(dtype=object)
    path = compute_global_path(map_array, epsilon_mm=20.0)
    print(path)

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

        # Create figure
        plt.ion()
        fig = plt.figure('Thymio')
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        x_data = [path[0][0]]
        y_data = [path[0][1]]
        lines = ax.plot(x_data, y_data, 'r.-')

        # Initialize state machine
        thymio.set_target(path[1][0], path[1][1])
        state: State = State.FOLLOW
        TARGET_TOLERANCE = 10 # mm

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

            # Global navigation

            # TODO Camera acquisition
            # TODO Update position with Kalman filter
            # thymio.set_pose(path[0][0], path[0][1], theta0)

            # Draw updated position
            x_data.append(thymio.x)
            y_data.append(thymio.y)
            lines[0].set_data(x_data, y_data)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()

            # Pace loop
            aw(thymio.client.sleep(0.2))

    plt.ioff()
    plt.figure('Final')
    plt.subplot(111)
    plt.gca().set_aspect('equal')
    plt.plot(x_data, y_data, 'r.-')
    plt.show()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
