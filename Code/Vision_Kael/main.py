import cv2
import numpy as np
from camera_setup import CameraStream
from feed_processing import createCanvasAndState

windowTitle = "Canvas view - q to quit"

def main():
    cam = CameraStream(index=0, width=1920, height=1080, fps=30).start()

    # make size of disp window smaller
    scale = 0.5

    try:
        while True:
            frame = cam.read()
            if frame is None:
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                continue

            # build canvas and state for this frame (encapsulates edges, aruco, zone, robot/goal, poses)
            canvas, state = createCanvasAndState(frame, robotId=8, goalId=9)

            # resize and show
            displayFrame = cv2.resize(canvas, (0, 0), fx=scale, fy=scale)
            cv2.imshow(windowTitle, displayFrame)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()