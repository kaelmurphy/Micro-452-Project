# Vision

## 1. Overview  
The vision program is called from our demo file using a single function: getVisionCoords(timeout=, showDisplay=)

- **timeout**: number of frames to wait before timing out  
- **showDisplay**: boolean controlling whether the live camera display is shown  

---

### 1.1. Vision acquisition pipeline  
The overall workflow used for vision acquisition is as follows:

#### 1.1.1. Capture camera feed  
The program initializes a camera stream and continuously reads frames for processing.

#### 1.1.2. Detect ArUco markers  
We use **six ArUco markers** to represent system components:

- **IDs 0–3** → Operating zone corners  
- **ID 8** → Robot  
- **ID 9** → Goal  

These IDs are constants within the program.  
(A more detailed explanation of ArUco code acquisition will be added in a later section.)

#### 1.1.3. Build the operating zone  
Once the corner ArUco codes (0–3) are detected, their centers are connected to form a polygon.  
This polygon defines the **operating zone**, which is the boundary that the robot must stay within.

#### 1.1.4. Compute homography  
After detecting the zone, we apply a **homography transform** (`cv2.getPerspectiveTransform`) to flatten the camera image into a top-down plane.  
This provides a coordinate system measured in **millimeters**, based on the physical dimensions we use for our zone.

#### 1.1.5. Mask robot and goal for obstacle detection  
Before performing obstacle detection, the robot and goal markers are removed from the obstacle image:

- The centers of ArUco IDs **8 (robot)** and **9 (goal)** are obtained.  
- A white circle is drawn over each marker (and the robot’s shadow) to prevent them from being picked up as obstacles.  
- Obstacle detection is performed on this masked image.

#### 1.1.6. Find robot orientation
We need to find the angle that the robot is at when we a querying the vision program. To do this, pass the pixel location of the center of the robot ArUco into the homography transform to return the point in mm. Then find the top left and the top right corners of the ArUco code of the robot, calculate the center point of these two using (TR - TL) / 2 where TR and TL are in pixel coordinates. Using the homography transform from earlier, we can then turn this point into world coordinates in mm. Drawing a line from the center of the ArUco code to this new point creates an angle with the horizontal axis produced by the homography transform.

#### 1.1.7. Obstacle detection
Once we have created the operating zone and gotten the angle of the robot, we check the image frame to see if there are any obstacles. The function detectObstacles(obsFrame, zone, minArea=) takes in the copy of the frame we made that has the white circles on it, the zone built for the corners, and the minimum area boundary for a valid obstacle in squared pixels. We do a lot of blurring, filtering, and masking, find the edges of the obstacles and draw contours on them. If the contours enclose an area greater than our minimum, an obstacle instance is added to the obstacle list which gets returned from this function.

#### 1.1.8. Drawing
Now that we have the pixel coordinates for our zone, robot, and goal, we draw them all using the draw helpers. 

#### 1.1.9. Coordinate conversion
Create an array that stores all the coordinates in pixel locations, we will then use the _extractFrameCoords function, which takes the frame, list of aruco codes, the operating zone, and whether there are obstacles and an H transform. First, take all the corners that get passed in through the ArUco list and call the pixelToWorld function to return mm coordinates, order matters, we start in the bottom left starting with ID 3 and work CCW to ID 0 in the top left, add all these new coordinates to the coordList to be returned. Next find robot and goal coordinates using the same pixelToWorld function. Finally, convert all the vertices of each obstacle to world coordinates using getWorldVerts function. 

#### 1.1.10. Stability filtering  
Before returning data to caller, the program verifies that our camera feed is stable. Individual frames might have errors, missing ArUco codes, or faults in obstacle detection. To avoid returning inconsitent or incorrect coordinates to the demo file, each frame's set of coordinates is added into a buffer. When the buffer contains `bufferLen = XX` similar coordinates in a row, we consider the vision capture to be stable. Once stable, the camera object and homography are cached for future queries.

#### 1.1.11. Final output  
After the stability check passes, the system returns four items:

1. A coordinate array in millimeters containing all corners, robot position, goal position, and all obstacle vertices.  
2. The robot’s initial orientation angle in the homographic world frame.  
3. The homography matrix used to convert pixels into millimeters.  
4. The pixel positions of the operating zone corners, used in the control hub live feed.

---

### 1.2. Fast robot pose acquisition  
Once the system has been initialized using `getVisionCoords(...)`, we assume the the global obstacles and the corners of our operating zone will not change. Since we are expecting this behaviour, it doesn't make sense to redo this processing every time we capture a frame. Instead, we created a lightweight function `getRobotPositionMm(...)` that we call whenever we want an updated robot position from the camera.

#### 1.2.1. getRobotPositionMm()  
This helper function reads a frame from the cached camera, finds the robot marker, and computes its world coordinates and orientation using the H transform from earlier. It returns the robot’s x-position, y-position, orientation angle, and a validity flag. If the robot is not visible, the function returns None for the position and angle, and False for `robot_seen`; if the robot is not in the frame the program will continue to run using only odometry.

---

### 1.3. Shutdown and live frame access  

#### 1.3.1. stopVision()  
When the demo finishes, we can shut down the vision system using the `stopVision()` function. This stops the background camera thread, releases the video capture handle, and closes all windows.

#### 1.3.2. getLiveFrameBGR()  
This function allows other parts of the program to request the most recent camera frame at any time. We overlayed the theoretical path overtop of the live feed so we can visually see how the robot is behaving.

---

## 2. Analysis

 This section is meant to be a **deep dive** into how the vision system works internally:

---

### 2.1. Camera acquisition and threading

#### 2.1.1. OpenCV video pipeline
- Explain how `cv2.VideoCapture(...)` interfaces with the OS camera driver.  
- Discuss frame format (BGR), resolution, FPS, and buffering.  
- Describe how properties like `CAP_PROP_FRAME_WIDTH`, `CAP_PROP_FRAME_HEIGHT`, and `CAP_PROP_FPS` are used.  
- Mention limitations: latency, dropped frames, driver-dependent behaviour.

#### 2.1.2. Threaded camera design (`CameraStream` class)  
- Explain why a background thread is used to continuously grab frames.  
- Discuss race conditions and why a `threading.Lock()` is needed around shared frame access.  
- Describe how `read()` returns a copy of the latest frame to avoid modification during processing.  
- Optional: include a small timing diagram of capture thread vs. processing loop.

#### 2.1.3. Camera warm-up and exposure behavior  
- Explain why the camera is “super saturated” at startup.  
- Discuss auto-exposure and auto-gain behaviour and why discarding initial frames stabilizes brightness.  
- If applicable, mention any manual exposure/gain control via OpenCV properties.

---

### 2.2. ArUco marker detection

#### 2.2.1. ArUco dictionaries and encoding  
- Explain what an ArUco marker is (binary square fiducial).  
- Describe how the dictionary (`cv2.aruco.getPredefinedDictionary(...)`) defines the set of valid IDs.  
- Briefly discuss error correction and why ArUco markers are robust to noise and partial occlusion.

#### 2.2.2. Detection steps (`detectAruco(...)`)  
- Outline the internal steps of `cv2.aruco.ArucoDetector.detectMarkers(...)`:  
  - image thresholding  
  - contour finding of candidate squares  
  - perspective normalization of each candidate  
  - binary decoding and dictionary matching  
- Explain the role of grayscale conversion (`cv2.cvtColor(..., COLOR_BGR2GRAY)`).  
- Explain how `corners` and `ids` are returned, and how you build `centerMap` and `cornerMap`.

#### 2.2.3. Marker coordinate conventions  
- Clarify the ordering of ArUco corners (e.g., top-left, top-right, bottom-right, bottom-left).  
- Explain how the center is computed as the average of the four corners.  
- Describe how consistent corner ordering is crucial for orientation computation and homography mapping.

---

### 2.3. Operating zone and coordinate frames

#### 2.3.1. Zone definition from markers 0–3  
- Explain how IDs 0–3 are mapped to physical board corners.  
- Describe the logic that builds `zone["corners"]` in a specific order (e.g., bottom-left, bottom-right, top-right, top-left).  
- Discuss why a consistent winding order (clockwise / counterclockwise) matters.

#### 2.3.2. World coordinate system (A0 sheet / board model)  
- Define the world coordinate frame in millimeters (origin, axes orientation).  
- Explain how the physical dimensions of the sheet are encoded (width, height).  
- Clarify how “bottom” and “top” of the board are defined relative to the camera view.

---

### 2.4. Homography and pixel→world mapping

#### 2.4.1. Homography fundamentals  
- Explain what a homography is: a 3×3 projective transform mapping points between planes.  
- Show the math:  
  - pixel point `p = [x, y, 1]^T`  
  - world point `p' ~ H * p`, followed by normalization.  
- Briefly derive the 8 degrees of freedom and why 4 point correspondences are sufficient.

#### 2.4.2. Computing `H` with `cv2.getPerspectiveTransform(...)`  
- Explain how the source points (pixel corners) and destination points (world mm corners) are constructed.  
- Show the mapping between zone corner order and world coordinates.  
- Mention any scaling or board dimension assumptions (e.g., using a slightly padded width/height).

#### 2.4.3. Using `pixelToWorld(...)` (`cv2.perspectiveTransform(...)`)  
- Explain how a single point is converted from pixels to world coordinates.  
- Discuss numerical issues (floating point, rounding) and how they affect obstacle vertices / robot coordinates.  
- Optionally, mention the inverse mapping (world→pixel) even if not used directly.

---

### 2.5. Robot pose estimation

#### 2.5.1. Position estimation  
- Explain how the robot’s pixel center is extracted from its ArUco marker and then passed to `pixelToWorld(...)`.  
- Discuss why using the marker center is a good approximation of the robot’s true reference point.

#### 2.5.2. Orientation estimation from top edge  
- Describe the geometric construction using the top-left and top-right corners of the marker.  
- Show how the midpoint of the top edge is computed in pixels, then mapped to world coordinates.  
- Explain how the orientation angle `θ` is obtained using `atan2(dy, dx)` and what each term represents.  
- Define the convention:  
  - `θ = 0` along +X axis (to the right)  
  - increasing `θ` counterclockwise  
- Mention wrap-around and normalization to `[0, 2π)` if applicable.

#### 2.5.3. Relationship to robot chassis frame  
- Explain how the ArUco marker’s printed orientation is aligned with the robot’s forward direction.  
- Discuss what happens if the marker is slightly misaligned physically and how that affects θ.

---

### 2.6. Obstacle detection and processing

#### 2.6.1. Color-space and masking choices  
- Describe why you operate in BGR and use `cv2.inRange(...)` on the blurred image.  
- If relevant, mention why HSV was or was not used.  
- Explain the chosen lower/upper color bounds and what physical colors they correspond to.

#### 2.6.2. Blurring and noise reduction (`cv2.GaussianBlur(...)`)  
- Explain Gaussian blur and how it reduces high-frequency noise.  
- Discuss kernel size selection and its impact on edge sharpness vs noise.

#### 2.6.3. Morphological operations (`cv2.morphologyEx(...)`)  
- Explain morphological closing and opening and why both are used in sequence.  
- Describe how these operations help fill small gaps and remove speckle noise.  
- Include basic set-theoretic intuition (dilation + erosion).

#### 2.6.4. Contour extraction and filtering (`cv2.findContours(...)`)  
- Explain how contours approximate object boundaries.  
- Describe `cv2.RETR_EXTERNAL` and why only external contours are used.  
- Discuss area filtering (`minArea`, `maxArea`) and how this removes noise and irrelevant blobs.

#### 2.6.5. Polygon approximation (`cv2.approxPolyDP(...)`)  
- Explain the Douglas–Peucker algorithm conceptually and how `epsilon` controls simplification.  
- Discuss why simplifying obstacle contours to fewer vertices is helpful for planning / collision checking.

#### 2.6.6. World-space obstacle representation (`Obstacle` class)  
- Explain how each obstacle’s contour is converted into a small set of vertices (`getWorldVerts(...)`).  
- Describe the CCW sorting and “bottom-left” starting point logic.  
- Mention how this representation is used later by the planner (convex/concave polygons, etc.).

---

### 2.7. Stability filtering and robustness

#### 2.7.1. Motivation for temporal filtering  
- Explain why single-frame detections are unreliable (lighting changes, dropped markers, noise).  
- Discuss typical failure modes observed during testing.

#### 2.7.2. Buffer design and similarity measure  
- Describe the buffer of recent coordinate arrays (`coordBuf`).  
- Explain the similarity function (e.g., matching obstacle IDs, vertex counts, and positions within a tolerance).  
- Justify the choice of buffer length `bufferLen` and tolerance values.

#### 2.7.3. Tradeoffs: latency vs stability  
- Discuss the tradeoff: longer buffer → more stable but more latency.  
- Explain how this impacts responsiveness of the control loop.  
- Mention any empirical tuning based on experiments.

---

### 2.8. Error sources and limitations

#### 2.8.1. Camera calibration and lens distortion  
- Note that without full camera calibration, there is residual distortion.  
- Explain how this can affect homography accuracy, especially near the edges of the frame.

#### 2.8.2. Lighting, reflections, and shadows  
- Discuss how changing lighting conditions can affect ArUco detection and obstacle segmentation.  
- Mention specific issues like glare on glossy surfaces or robot shadows.

#### 2.8.3. Occlusions and marker visibility  
- Explain what happens when the robot occludes markers or obstacles overlap markers.  
- Discuss how the system currently handles partial failure (e.g., missing one corner marker).

#### 2.8.4. Spatial and temporal resolution limits  
- Connect camera resolution and field of view to minimum resolvable obstacle size.  
- Discuss FPS limits and how they bound how quickly the robot can safely move.

---

### 2.9. Computational complexity and performance

#### 2.9.1. Per-frame complexity breakdown  
- Roughly estimate complexity for each pipeline stage:  
  - ArUco detection, homography mapping, obstacle processing, etc.  
- Indicate which stages dominate runtime.

#### 2.9.2. Measured performance (if available)  
- Suggest adding experimental timing measurements for:  
  - frame acquisition  
  - `detectAruco(...)`  
  - `detectObstacles(...)`  
  - `getRobotPositionMm(...)`  
- Discuss whether the system is CPU-bound or I/O-bound.

#### 2.9.3. Possible optimizations  
- Mention potential improvements:  
  - ROI-based processing instead of full-frame  
  - downsampling for obstacle detection  
  - using GPU-accelerated OpenCV operations  
  - caching more intermediate results

---
## 3. Decision Making
