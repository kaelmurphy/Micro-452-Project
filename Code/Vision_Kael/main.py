import cv2
from camera_setup import CameraStream
from feed_processing import createCanvasAndState

windowTitle = "Canvas view - q to quit"

def main():
    cam = CameraStream(index=0, width=1920, height=1080, fps=30).start()
    try:
        while True:
            frame = cam.read()
            if frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            canvas, state = createCanvasAndState(frame)
            cv2.imshow(windowTitle, cv2.resize(canvas, (0, 0), fx=0.5, fy=0.5))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()