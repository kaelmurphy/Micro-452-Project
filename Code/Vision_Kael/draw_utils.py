import cv2
import numpy as np

def drawOperatingZone(frame, zone, color=(0, 255, 255)):
    if not zone.get('corners'):
        return frame
        
    pts = np.array(zone['corners'], dtype=np.int32)
    overlay = frame.copy()
    
    cv2.fillPoly(overlay, [pts], color)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)
    cv2.polylines(frame, [pts], True, color, 3)
    return frame