import sys
import os
import time
import numpy as np
import cv2

# Add src directory to path
# _src_path = os.path.join(os.path.dirname(__file__), 'src')
# if _src_path not in sys.path:
#     sys.path.insert(0, _src_path)

from .src import camera_setup
from .src import aruco_utils
from .src import coord_utils
from .src import obstacle
from .src import feed_processing
from .src import draw_utils

def getCoordinatesFromVision(timeout=None, show_display=True):
    """
    Get coordinate data from the vision system.
    
    Args:
        timeout: Maximum time to wait for stability (None = wait forever)
        stabilityFrames: Number of consecutive stable frames required
        show_display: Whether to show live video feed (default: False)
    
    Returns:
        numpy array with columns [type, id, label, x, y]
        Empty array if failed or timeout
    """
    def coords_are_similar(arr1, arr2, tol=20.0):
        if arr1.shape != arr2.shape:
            return False
        # Only compare x_mm and y_mm columns (last two columns)
        xy1 = arr1[:, -2:].astype(float)
        xy2 = arr2[:, -2:].astype(float)
        return np.all(np.abs(xy1 - xy2) < tol)
    bufferLen = 3  # Require 3 stable frames for robust output
    coordsBuffer = []
    camera = None
    try:
        # Import inside function to avoid linting issues
        

        ROBOT_ID = 8
        GOAL_ID = 9
        ORIGIN_ID = 3

        camera = camera_setup.CameraStream(width=640, height=480, fps=30)
        camera.start()
        time.sleep(1)

        if show_display:
            cv2.namedWindow('Vision System', cv2.WINDOW_AUTOSIZE)

        startTime = time.time()
        while True:
            if timeout is not None:
                elapsed = time.time() - startTime
                if elapsed > timeout:
                    if camera is not None:
                        camera.stop()
                    return np.array([]).reshape(0, 5)

            frame = camera.read() if camera is not None else None
            if frame is None:
                time.sleep(2)
                continue

            display_frame = frame.copy()
            try:
                ids, centers, cornersMap, _ = aruco_utils.detectAruco(frame)
                zone = aruco_utils.buildOperatingZone(centers)
                originPx = centers.get(ORIGIN_ID)

                # Draw overlays if requested
                if show_display:
                    if zone.get('isComplete') and zone.get('corners'):
                        display_frame = draw_utils.drawOperatingZone(display_frame, zone, color=(0, 255, 255))
                    if ROBOT_ID in centers:
                        draw_utils.drawRobotMarker(display_frame, centers[ROBOT_ID], cornersMap.get(ROBOT_ID))
                    if GOAL_ID in centers:
                        draw_utils.drawGoalMarker(display_frame, centers[GOAL_ID])
                    if zone.get('isComplete'):
                        obstacles = obstacle.detectObstacles(frame, zone, minArea=1000)
                        if obstacles:
                            display_frame = obstacle.drawObstacles(display_frame, obstacles, cornersMap, originPx, False, False)
                    cv2.imshow('Vision System', display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        if camera is not None:
                            camera.stop()
                        cv2.destroyAllWindows()
                        return np.array([]).reshape(0, 5)

                # Always extract and return coordinates in mm
                coords = _extractCoordinatesFromFrame(frame, centers, cornersMap, zone, originPx)
                print(coords)
                # Only start stabilization if all required elements are detected
                types = coords[:,0] if coords.size > 0 else []
                has_corners = np.sum(types == 'corner') >= 4
                has_robot = np.any(types == 'robot')
                has_goal = np.any(types == 'goal')
                has_obstacle = np.any(types == 'obstacle')
                # For obstacles, compare by obstacle index and vertices
                def obstacles_are_similar(obs_arr1, obs_arr2, tol=50.0):
                    if len(obs_arr1) != len(obs_arr2):
                        return False
                    for row1, row2 in zip(obs_arr1, obs_arr2):
                        if row1[2] != row2[2] or row1[1] != row2[1]:
                            return False
                        x1, y1 = round(row1[3], 1), round(row1[4], 1)
                        x2, y2 = round(row2[3], 1), round(row2[4], 1)
                        if abs(x1 - x2) > tol or abs(y1 - y2) > tol:
                            return False
                    return True
                if coords.size > 0 and has_corners and has_robot and has_goal and has_obstacle:
                    # Separate obstacle rows for comparison
                    obstacle_rows = [row for row in coords if row[0] == 'obstacle']
                    if len(coordsBuffer) == 0 or obstacles_are_similar(obstacle_rows, [row for row in coordsBuffer[-1] if row[0] == 'obstacle']):
                        coordsBuffer.append(coords.copy())
                        if len(coordsBuffer) > bufferLen:
                            coordsBuffer = coordsBuffer[-bufferLen:]
                        # If all buffered arrays are similar, or if bufferLen reached and all elements are present, return immediately
                        if (len(coordsBuffer) == bufferLen and all(obstacles_are_similar(
                            [row for row in c if row[0] == 'obstacle'],
                            [row for row in coordsBuffer[0] if row[0] == 'obstacle']) for c in coordsBuffer)) or (len(coordsBuffer) == bufferLen and has_corners and has_robot and has_goal and has_obstacle):
                            if camera is not None:
                                camera.stop()
                            if show_display:
                                cv2.destroyAllWindows()
                            return coordsBuffer[0]
                    else:
                        coordsBuffer = [coords.copy()]
            except Exception as e:
                print(f"[DEBUG] Exception in main loop: {e}")
    except Exception:
        try:
            if camera is not None:
                camera.stop()
            if show_display:
                cv2.destroyAllWindows()
        except Exception:
            pass
        return np.array([]).reshape(0, 5)


def _statesAreSimilar(state1, state2, tolerance=10):
    """Check if two states are similar for stability detection."""
    if not state1 or not state2:
        return False
    
    if state1.get('robot') and state2.get('robot'):
        r1, r2 = state1['robot'], state2['robot']
        if abs(r1[0] - r2[0]) > tolerance or abs(r1[1] - r2[1]) > tolerance:
            return False
    elif state1.get('robot') != state2.get('robot'):
        return False
    
    if state1.get('goal') and state2.get('goal'):
        g1, g2 = state1['goal'], state2['goal']
        if abs(g1[0] - g2[0]) > tolerance or abs(g1[1] - g2[1]) > tolerance:
            return False
    elif state1.get('goal') != state2.get('goal'):
        return False
    
    if abs(state1.get('obstacles', 0) - state2.get('obstacles', 0)) > 0:
        return False
    
    return True

def _extractCoordinatesFromFrame(frame, centers, cornersMap, zone, originPx):
    """Extract all coordinates from a stable frame."""
    
    ROBOT_ID = 8
    GOAL_ID = 9
    coordinates_list = []
    
    try:
        if zone.get('isComplete'):
            corners_px = zone['corners']
            corner_labels = ['poly0'] * 4
            corner_names = ['3', '2', '1', '0']
            corners_world = feed_processing.convertPixelsToWorldCoords(corners_px, originPx, cornersMap)
            if isinstance(corners_world, (list, tuple, np.ndarray)) and len(corners_world) > 0 and hasattr(corners_world[0], '__iter__'):
                for (x, y), poly_id, label in zip(corners_world, corner_labels, corner_names):
                    coordinates_list.append(['corner', poly_id, label, float(x), float(y)])
        if ROBOT_ID in centers:
            rCx, rCy, robotTheta = coord_utils.robotWorldPose(centers, cornersMap, ROBOT_ID)
            if rCx is not None and rCy is not None:
                robot_world = feed_processing.convertPixelsToWorldCoords((rCx, rCy), originPx, cornersMap)
                if isinstance(robot_world, tuple) and len(robot_world) == 2:
                    coordinates_list.append(['robot', 'robot', '0', float(robot_world[0]), float(robot_world[1])])
        if GOAL_ID in centers:
            goal_world = feed_processing.convertPixelsToWorldCoords(centers[GOAL_ID], originPx, cornersMap)
            if isinstance(goal_world, tuple) and len(goal_world) == 2:
                coordinates_list.append(['goal', 'goal', '0', float(goal_world[0]), float(goal_world[1])])

        obstacles = obstacle.detectObstacles(frame, zone, minArea=1000)
        if obstacles is None:
            obstacles = []
        for obs_idx, obstacle_obj in enumerate(obstacles):
            world_vertices = obstacle_obj.getVerticesWorldCoords(cornersMap, originPx)
            if isinstance(world_vertices, tuple) and len(world_vertices) == 2:
                world_vertices = [world_vertices]
            if isinstance(world_vertices, np.ndarray):
                world_vertices = world_vertices.tolist()
            poly_id = f"poly{obs_idx+1}"
            for v_idx, (x, y) in enumerate(world_vertices):
                coordinates_list.append([
                    'obstacle',
                    poly_id,
                    str(v_idx + 1),
                    float(x),
                    float(y)
                ])

        print(" break ")
        return np.array(coordinates_list, dtype=object) if coordinates_list else np.array([]).reshape(0, 5)

    except Exception:
        print("No coordinates extracted from frame due to error.")
        return np.array([]).reshape(0, 5)


# Example usage and testing
if __name__ == "__main__":
    print("=" * 40)
    
    coordinates = getCoordinatesFromVision(timeout=None, stabilityFrames=20000)
    
    if coordinates.size == 0:
        print("No coordinates found or timeout occurred")
    else:
        print("Returned coordinates array:")
        print(coordinates)