import cv2
from camera_setup import CameraStream
from feed_processing import createCanvasAndState, resetStability

def main():
    cam = CameraStream(index=0, width=3840, height=2160, fps=30).start()
    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue
                
            canvas, state = createCanvasAndState(frame)
            # use higher quality interpolation and larger scale for clearer display
            display_frame = cv2.resize(canvas, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_LANCZOS4)
            cv2.imshow("q = quit, r = reset", display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                resetStability()
                
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()