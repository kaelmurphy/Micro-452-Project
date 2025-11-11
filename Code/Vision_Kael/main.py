import cv2
from camera_setup import CameraStream
from feed_processing import detectEdges, overlayEdges, detectAruco, buildOperatingZone, drawOperatingZone

windowTitle = "Aukey 1080p (q quit, e edges, t toggle view, a aruco, z zone)"

def main():
    cam = CameraStream(index=1, width=1920, height=1080, fps=30).start()

    # make size of disp window smaller
    scale = 0.5

    # disp settings on init
    showEdgeMap = False
    showEdges = False
    showAruco = True
    showZone = True 

    try:
        while True:
            frame = cam.read()
            if frame is None:
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                continue
            
            # create disp that shows edge detection on black background
            edges = detectEdges(frame, low=25, high=80, blur=3)

            # select disp
            if showEdgeMap:
                displayFrame = edges
                arucoBase = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # for aruco/zone drawing
            else:
                displayFrame = frame
                if showEdges:
                    displayFrame = overlayEdges(displayFrame, edges)
                arucoBase = displayFrame

            # aruco detect
            ids, centers, anno = detectAruco(arucoBase, dictName="DICT_4X4_50", draw=True) if showAruco else ([], {}, arucoBase)

            # zone overlay
            if showZone:
                zone = buildOperatingZone(centers)
                anno = drawOperatingZone(anno, zone, color=(0, 255, 255))
                # info on missing zones
                hud = f"{'ok' if zone['isComplete'] else 'missing_zones: ' + ','.join(map(str, zone['missing']))}"
                cv2.putText(anno, hud, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

            # if want edges make it grayscale
            if showEdgeMap:
                displayFrame = cv2.cvtColor(anno, cv2.COLOR_BGR2GRAY)
            else:
                displayFrame = anno

            displayFrame = cv2.resize(displayFrame, (0, 0), fx=scale, fy=scale)
            cv2.imshow(windowTitle, displayFrame)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('e'):
                showEdges = not showEdges
            elif k == ord('t'):
                showEdgeMap = not showEdgeMap
            elif k == ord('a'):
                showAruco = not showAruco
            elif k == ord('z'):
                showZone = not showZone

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()