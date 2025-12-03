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

### 2.1. Camera acquisition and threading

#### 2.1.1. OpenCV video pipeline

The camera is accessed through OpenCV’s `VideoCapture` using a fixed camera index (0). `VideoCapture.read()` pulls frames and converts them into a NumPy array, which will be accessed by our program.

When making the CameraStream object we use:

- Frame width and height: 640×480
- Framerate: 30 fps

#### 2.1.2. Threaded camera (CameraStream) design

Instead of calling `read()` in the vision processing loop, the program uses the `CameraStream` class that does the acquisition in its own thread. Essentially:

- The camera thread continuously pulls frames from the Camera and caches the latest frame.
- The vision logic program reads from that cached frame whenever it needs.

We initialize the camera once and keep a single CameraStream running in the background continuously grabbing the latest frame. Different functions can then access this shared stream without opening or setting up the camera themselves. For example, we have slow, heavy processing in getVisionCoords(...), while getLiveFrameBGR(...) only requires the raw frame with no additional processing. In our current main structure, these calls occur sequentially, and we wait for getVisionCoords(...) to return. However, the design supports a parallel architecture where a fast function can read new frames without being blocked by heavier functions. Later in the program, this becomes more important: functions like getRobotPositionMm(...) and getLiveFrameBGR(...) can grab the most recent frame from the camera, without having to reopen the camera or wait for another function to finish running. Even though we don’t really capitalise on this to parallelize more of our program, the cached-camera design is the most robust solution.

CameraStream class:

- The class has:
  - `self.cap`: the `VideoCapture` object tied to the camera.
  - `self.frame`: the newest BGR frame captured.
  - `self.lock`: a `threading.Lock` protecting access to `self.frame`.
  - `self.running`: a flag that controls the capture loop.
  - `self.thread`: the background thread object.

- The `_update` method:
  - Runs in a `while self.running` loop.
  - Calls `cap.read()` to grab the next frame.
  - Acquires lock and updates `self.frame`.
  - Small sleep timer to avoid using 100% of the CPU.

- The `read` method:
  - Acquires the same lock.
  - Returns a `.copy()` of the frame buffer.

What’s the point of the lock?:

- The main thread might try to read the frame at the same time as the camera thread is writing a new one.
- Without coordination, this could produce torn frames or corrupt data.

<p align="center">
    <img src="../images/python_locking.png" alt="python_locking" width="400"/>
</p>

<p style="text-align: center;">
 As shown above the lock makes sure that only one thread can access the shared <code>self.frame</code> array at a time. When a thread calls <code>lock.acquire()</code>, it either takes the lock or blocks until the other thread releases it. The main thread acquires the lock, copies <code>self.frame</code>, and releases it so the camera thread can keep capturing. The camera thread acquires the lock, writes the new frame, then releases it. As a result, the main thread never sees a partially written frame, and the camera thread never overwrites the array while it is being copied.
</p>

Even with the lock, once <code>read()</code> returns the main thread may spend some time processing the frame (for example, during ArUco detection). If we returned a reference to <code>self.frame</code> instead of <code>self.frame.copy()</code>, the camera thread might modify that array while the program is still processing it.

#### 2.1.3. Camera warm-up and exposure behavior

If we immediately started running ArUco detection on the unstable frames generated right after the camera opens, the early detections would be inconsistent and delay the rest of our vision processing program. To avoid this, we discard the first chunk of frames captured after opening the camera. While discarding we initiate a small sleep per frame.

```python
warmup_frames = 30
for _ in range(warmup_frames):
  ok, _ = self.cap.read()
  if not ok:
    break
  time.sleep(0.02)
```

This does two things:

1. Gives the camera enough time to stabilize exposure, gain, and white balance.
2. Avoid over-using the CPU, similar to how we used the sleep timer in the `_update` method earlier.

After exiting the for-loop, the frames are closer to their steady state, which improves ArUco and obstacle detection later in the program.

---

### 2.2. ArUco marker detection

#### 2.2.1. ArUco dictionaries and encoding

An ArUco marker is a QR-code-esque identifier typically used for computer vision applications:

- A black border defines the region.
- The interior is a binary grid encoding an ID.
- It has redundancy and error correction:
  - If some cells are partially blocked or corrupted by noise, the decoder can still recover the ID.
  - The orientation of the marker can be deduced since the binary pattern only matches one orientation.

<p align="center">
    <img src="../images/Aruco_example.png" alt="Aruco" width="200"/>
</p>

<p style="text-align: center;">
 For example above is the ArUco code in the 4X4 dictionary for ID 0
</p>

The dictionary that we use ("DICT_4X4_50") contains:

- The size of the inner grid (4x4).
- The number of valid patterns and how they map to IDs, in our case 0 -> 49.
- Each marker pattern is different enough from the others that the camera doesn't confuse them.

In our system:

- IDs 0–3 are the operating zone corners [bottom-left = 3, bottom-right = 2, top-right = 1, top-left = 0].
- ID 8 is the robot.
- ID 9 is the goal.

#### 2.2.2. Detection steps (`detectAruco`)

The `detectAruco` function implements the ArUco detection pipeline, this is all internal to OpenCV so I didn't have to write the code for this, but I will explain it below.

---

##### 1. **Conversion to greyscale**

ArUco markers only store information in black & white intensity as seen above, so OpenCV turns the three BGR channels into a single greyscale channel:

$$
I(x,y) = 0.114\,B(x,y) + 0.587\,G(x,y) + 0.299\,R(x,y)
$$

Where:

- $I(x,y)$: greyscale intensity at pixel (x,y)
- $B(x,y),G(x,y),R(x,y)$: blue, green, and red channel intensities at pixel (x,y)
- $0.114, 0.587, 0.299$: channel coefficients used by OpenCV

I found the coefficients in the OpenCV documentation at:  
https://docs.opencv.org/4.x/de/d25/imgproc_color_conversions.html

**Why convert to greyscale:**  
- ArUco markers only encode information through black & white intensity, not colour. Using greyscale makes the filtering process easier since the algorithm only separates “dark” from “light”,  rather than processing three colour channels.
- Greyscale removes colour-specific noise and produces cleaner edges for thresholding and contour detection.

---

##### 2. **Detector initialization**

OpenCV loads the `DICT_4X4_50` dictionary.  
This dictionary restricts detection to exactly 50 known 4×4 bit patterns (IDs 0–49)

---

##### 3. **Looking for ArUco markers**

###### a. **Adaptive thresholding**

OpenCV uses a local mean threshold for each pixel to compensate for uneven image lighting:

$$
T(x,y) = \frac{1}{|N|} \sum_{(u,v)\in\N} I(u,v) - C
$$

Where:

- $T(x,y)$: threshold at pixel (x,y)
- $N$: local neighborhood window
- $|N|$: number of pixels in the neighborhood
- $I(u,v)$: greyscale intensity at neighbor pixel (u,v)
- $C$: constant offset to bias the threshold

---

###### b. **Contour filtering**

After thresholding, the image is binarized with pixels either being black (0) or white (255). OpenCV find areas of the image containing boundaries between white and black regions, it "walks" along these edges and returns its path (the contour). Most contours in the image are not ArUco markers, so OpenCV filters contours based on:

- area  
- convexity  
- approximate shape  

---

###### c. **Warping and bit sampling**

Initially the contour is tilted / skewed, OpenCV does a homographic transform using the 4 corners of the contour, knowing that it is a perfect square when orthogonal to the camera view. The H-transform turns each contour into a perfect square:

$$
\begin{bmatrix}
x' \\[4pt]
y' \\[4pt]
w'
\end{bmatrix}
=
H
\begin{bmatrix}
x \\[4pt]
y \\[4pt]
1
\end{bmatrix},
\qquad
H =
\begin{bmatrix}
h_{00} & h_{01} & h_{02} \\
h_{10} & h_{11} & h_{12} \\
h_{20} & h_{21} & h_{22}
\end{bmatrix}
$$

$$
x_{\text{norm}} = \frac{x'}{w'}, 
\qquad
y_{\text{norm}} = \frac{y'}{w'}
$$

$$
x_{\text{norm}}
=
\frac{
h_{00}x + h_{01}y + h_{02}
}{
h_{20}x + h_{21}y + h_{22}
},
\qquad
y_{\text{norm}}
=
\frac{
h_{10}x + h_{11}y + h_{12}
}{
h_{20}x + h_{21}y + h_{22}
}.
$$

Where the homography parameters are defined as:

- $h_{00}, h_{01}$: control horizontal scaling, rotation, and skew  
- $h_{10}, h_{11}$: control vertical scaling, rotation, and skew  
- $h_{02}, h_{12}$: control translation (shifting the warped image)  
- $h_{20}, h_{21}$: encode perspective tilt (how much lines "converge")  
- $h_{22}$: normalization term (usually set to 1)

And $w'$ is a scale factor used to turn the coordinates back into pixel coordinates after the transform.

Homography computation:

Given 4 corner points in the image $(x_i, y_i)$ and the 4 corner points of a perfect square $(X_i, Y_i)$, the transform satisfies:

$$
\begin{bmatrix}
X_i \\ Y_i \\ 1
\end{bmatrix}
\sim
H
\begin{bmatrix}
x_i \\ y_i \\ 1
\end{bmatrix}.
$$

This makes 8 linear equations (2 per corner). OpenCV solves for the 8 unknowns in $H$ (with $h_{22}=1$) internally using a method called a "Direct Linear Transform". OpenCV then samples the sub-pixel values returned from the homography transform using bilinear interpolation:

$$
I(x,y) = 
w_{00} I(\lfloor x \rfloor,\lfloor y \rfloor)
+ w_{01} I(\lfloor x \rfloor,\lfloor y \rfloor+1)
+ w_{10} I(\lfloor x \rfloor+1,\lfloor y \rfloor)
+ w_{11} I(\lfloor x \rfloor+1,\lfloor y \rfloor+1)
$$

Where:

- $I(x,y)$: interpolated intensity at fractional coordinate (x,y)
- $\lfloor x \rfloor,\lfloor y \rfloor$: nearest integer pixel locations
- $w_{ij}$: interpolation weights (sum to 1)

The homography transform paired with the bilinear interpolation returns the intensity value of the sub-pixel brightness. This will be used to determine whether this sub-pixel is apart of the ArUco code.

---

###### d. **Dictionary matching**

The sampled 4×4 bits form a candidate pattern $M$. OpenCV compares this against each dictionary entry $D_k$ to ensure that only one ID is matched to what we have identified:

$$
d_H(M, D_k) = \text{number of bit positions where } M \neq D_k
$$

Where:

- $M$: extracted 4×4 bit matrix
- $D_k$: k-th dictionary pattern
- $d_H$: Hamming distance (bit mismatches)

---

##### 4. **Output structure**

The detector returns <code>ids</code>, a list of IDs that were found in the frame, and <code>corners</code>, a list containing the corners for each of the aruco markers in pixel-coordinates.

We compute the marker center as the mean of the four corners:

$$
c = \frac{1}{4} \sum_{k=0}^{3} p_k
$$

Where:

- $p_k$: k-th corner point (2D pixel coordinate)
- $c$: computed marker center

---

### 2.3. Operating zone and coordinates

#### 2.3.1. Zone definition from markers 0–3

The operating zone for our robot is defined by the four corner ArUco markers:

- IDs 0, 1, 2, 3 sit at the corners of the physical board.
- The code constructs a `zone` dictionary where:
  - `zone["isComplete"]` is true when all corners are detected.
  - `zone["missing"]` is a list of the missing corner IDs.
  - `zone["corners"]` is an ordered list of the pixel centers of the corners marker.

The order is forced even if the IDS are returned in a different sequence because:

1. Homography transform assumes that the nth pixel corner corresponds to the nth world corner. If the order is wrong, the homography will warp the board incorrectly.
2. When we draw the polygon representing the zone, we want a simple counter-clockwise loop that does not self-intersect; consistent ordering guarantees that.
3. When we return the list to <code>demo.py</code>, the global navigation program expects all polygons to be CCW.

---

#### 2.3.2. World coordinate system

The world coordinate system is in millimeters, and the size of it is set by the physical dimensions we chose to use on our background sheet of paper. The known width and height of our operating zone are set as constants:

- `widthMm` ~ 1255 mm
- `heightMm` ~ 740 mm

The coordinates of the four board corners in world space are:
  - (0, 0) = bottom-left
  - (widthMm, 0) = bottom-right
  - (widthMm, heightMm) = top-right
  - (0, heightMm) = top-left

All points inside the operating zone can then be expressed in millimeters after finding the suitable homographic transform.

---

### 2.4. Homography and pixel to world  coordinate mapping

#### 2.4.1. Homography transform

As discussed earlier in 2.2.2.3, the homography transformation accounts for when the camera is not perfectly orthogonal to the oeprating zone, if there is any rotation of the board, and scaling differences that happen as you move out of the center of the camera's vision. For our project the problem was that we needed our operating zone to have a uniform scale so that our Thymio could be sent waypoints in millimeters anywhere in the zone, the H transform solves this:

<p align="center">
    <img src="../images/homography_easy.png" alt="Homogrpahy" width="400"/>
</p>

Similarly to the ArUco codes:
- The zone corner pixel centers (zone["corners"]) are the source points.
- The world corner coordinates are the destination points.
- `cv2.getPerspectiveTransform` takes these and solves for H.

To map a pixel point pt = (x, y) into the the millimeter world coordinates we form a 1×1×2 array to pass into `cv2.perspectiveTransform`, OpenCV multiplies H by [x, y, 1]^T internally and returns the result, finally it normalizes by the third coordinate W' to yield (X, Y). It returns (X_mm, Y_mm) which is the position in millimeters. We also use the inverse of the H transform to convert our path (in mm) back into pixel coordinates for our display hub. 

---

### 2.5. Robot pose estimation

#### 2.5.1. Robot position and orientation

The robot has an ArUco marker with ID 8 and using `centerMap[8]` we get the pixel center. To convert to world position we use `pixelToWorld(center_pixel, H)` and the robot’s estimated position (x_mm, y_mm) is returned. We find the orientation of the robot using the center of the top edge as the forward direction. Using the ArUco corner ordering convention we:

1. Take the corners for ID 8 from `cornerMap`, index 0 is top-left and index 1 is top-right.
2. Compute the midpoint of these two corners in pixel coordinates.
3. Convert:
   - The marker center to world coordinates: (cx, cy).
   - The midpoint of the top edge to world coordinates: (tx, ty).
4. Create a vector from robot-center to edge-midpoint:
   - dx = tx - cx
   - dy = ty - cy
5. Compute the orientation angle:
   - $\theta$ = atan2(dy, dx)

$\theta$ is defined in the world coordinates by:

- $\theta$ = 0 when the robot faces along +X (parallel to zone bottom edge facing the right).
- $\theta$ increases counter-clockwise:
  - $\pi/2$: robot faces +Y (towards the top of the board).
  - $\pi$: robot faces -X (leftwards).
  - $3\pi$/2: robot faces -Y (downwards).

The ArUco marker is attached to the robot so that its center is roughly in the same position as the robot’s center of rotation and its top edge points in the robot’s forward direction. If the marker is misaligned then $\theta$ will have a constant offset with the real forward heading of the robot.

---

### 2.6. Obstacle detection and processing

#### 2.6.1. Color-space and masking choices

Obstacle detection is performed directly in BGR space with a guassian blur that reduces noise and `cv2.inRange` is used with a lower and upper threshold on the B, G, and R channels. We chose to stay in BGR rather than convert to HSV for simplicity, it was easier to trial and error the right colour range for obstacle detection than transform everything to the HSV space.

insert photo here

Additionally, the system uses a binary zone mask; it builds a filled polygon from the zone corners to zero out everything outside the operating zone and so that objects or noise outside the board do not show up as obstacles. 

Before obstacle detection, the robot and goal markers are painted over with white circles on the obstacle frame:

insert photo here

We also use a paint-over to stop the ArUco markers and the robot's shadow from being detected as obstacles. The radius was chosen based on the size of the box created by the marker’s corners, it was scaled to cover the marker and nearby shadows.

#### 2.6.2. Blurring

A Gaussian blur is applied to the BGR image before thresholding using the colour channels. Gaussian blur replaces each pixel with a weighted average of its neighbors, it smooths out small changes in intensity and color due to sensor noise or texture. Finally, it limits the amount of isolated pixels that may be classified as an obstacle. The kernel size (5x5) was chosen because it is large enough to smooth out noise at the pixel scale, but it is small enough to keep the edges of obstacles.

<p align="center">
    <img src="../images/gaussian_blur.png" alt="Gaussian" width="400"/>
</p>

$$
I_{\text{blur}}(x,y)
=
\sum_{i=-2}^{2}
\sum_{j=-2}^{2}
G_{\text{5×5}}(i,j)\, I(x+i,\, y+j)
\qquad
G_{\text{5×5}} =
\frac{1}{273}
\begin{bmatrix}
1 & 4 & 7 & 4 & 1 \\
4 & 16 & 26 & 16 & 4 \\
7 & 26 & 41 & 26 & 7 \\
4 & 16 & 26 & 16 & 4 \\
1 & 4 & 7 & 4 & 1
\end{bmatrix}
$$

We apply a $5\times 5$ Gaussian blur by taking each pixel and combining it with the surrounding twenty-four neighbors using the fixed weights supplied by OpenCV. The weight matrix approximates a Gaussian distribution. For each pixel, we multiply every neighbor by the corresponding weight and add the results. The weights are such that they sum to one, which keeps the image brightness the same.

---

#### 2.6.3. Colour thresholding

After blurring, obstacle detection is performed by applying cv2.inRange directly in the BGR color space. This function compares each pixel against a lower and upper BGR bound chosen earlier:

<code>mask = cv2.inRange(blurred, lowerBGR, upperBGR)</code>

Pixels with Blue, Green, and Red channel values all within the ranges are set as 255, and everything else else is set as 0. This is the same method we used to create our binary mask used in the greyscale detection used for the ArUco code, it then gets passed into our morphological functions where the mask will be cleaned up so our contour detection returns the obstacles we are interested in. 

#### 2.6.4. Morphological operations and findContours

After thresholding we do some cleaning-up of the mask using:

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
```
The elliptical kernel defines the neighborhood used for smoothing. An ellipse is chosen because it produces natural-looking shapes that match our obstacles better than a square kernel. The first operation is closing, which fills small holes and connects small breaks so that each obstacle turns into a solid region. Closing is applied twice to ensure that obstacles broken by thresholding are repaired. The second operation is opening, which removes small isolated noise pixels and smooths the edges of the cleaned regions.

We then extract the outline of each obstacle using <code>findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)</code>:

- Retrieval mode: external contour only which returns the outer boundary of each obstacle and ignores any holes.
- Approximation method: chain approximation reduces the number of stored boundary points and keeps only the essential outline.

Each contour represents an obstacle. The system then computes the area of each contour and filters them using two thresholds:

- A minimum area (minArea).
- A maximum area (maxArea).

Small areas usually correspond to noise, or artifacts, and large areas mean that there is likely lighting problems and large sections of our zone are being detected. Contours within the acceptable range are turned into Obstacle objects and added to the list.

#### 2.6.6. Polygon approximation

Each `Obstacle` object simplifies its contour into a smaller polygon using the <code>approxPolyDP(self.contour, epsilon, True)</code>:

```python
if self._verts is None:
  peri = cv2.arcLength(self.contour, True)
  epsilon = 0.02 * peri
  approx = cv2.approxPolyDP(self.contour, epsilon, True)

  pts = [(float(p[0][0]), float(p[0][1])) for p in approx]
```

Where:
- peri is the perimiter length of the contour.
- epsilon is the maximum deviation allowed for the approximation.
- approx is the new simplified polygon.
- pts is the list of (x,y) in pixel coordinates.

Benefits of using a simplified polygon approximation are:

- Eliminates unnecessary detail in the contour, small variations on the boundary are removed.
- Produces obstacle polygons with a reasonable number of vertices.
- Gives stable representation of obstacle vertices for repeated trials.

After approximation, the obstacle vertices are “snapped” to a pixel grid. Each corner is rounded to the nearest multiple of a small grid spacing (we used 3), and vertices that end up close are merged to further simplify the obstacle:

```python
grid = 3
snapped = [
    (int(round(x / grid) * grid), int(round(y / grid) * grid))
    for (x, y) in pts
]

merged = []
mergeDist = 3
for x, y in snapped:
    if not merged:
        merged.append([x, y])
        continue

    found = False
    for v in merged:
        dx = x - v[0]
        dy = y - v[1]
        if dx * dx + dy * dy <= mergeDist * mergeDist:
            v[0] = int(round((v[0] + x) / 2))
            v[1] = int(round((v[1] + y) / 2))
            found = True
            break

    if not found:
        merged.append([x, y])

self._verts = [(vx, vy) for vx, vy in merged]
```

#### 2.6.7. Obstacle class

Once the polygon vertices are defined in pixel coordinates, we convert them to world coordinates using the same homography H used for markers. `pixelToWorld` is called and the resulting world coordinates are added to a list. The vertices are then sorted by:

1. Computing the centroid.
2. For each vertex, compute the angle from centroid to vertex.
3. Sort by this angle to obtain counter-clockwise order of vertices.
4. Find the bottom-left vertex (lowest Y, then lowest X), and rotate the list so that this vertex is first.

Why this ordering?

1. Consistent vertex labeling regardless of detection order.
2. We use Shapely to draw the polygons for path planning, and it requires the polygons to have its vertices defined in counter-clockwise. This is expanded on further in the global navigation section.

The final representation of an obstacle in the coordinate array is as a set of rows:

- type: "vertex"
- polygon ID: "polyX"
- vertex label: "1", "2", ...
- x_mm, y_mm: world coordinates in millimeters

---

### 2.7. Stability filtering and robustness

#### 2.7.1. Motivation for temporal filtering

The ArUco detector occasionally misses a marker if it is partially blocked or blurry and obstacle extraction can be sensitive to lighting changes. If we returned vision coordinates using one frame:

- Corners may disappear and reappear.
- Obstacles could jump around.
- Inconsistent robot and goal positions and orientations.

To avoid this, we used a buffer that stores frames and compares the coordinate values to see if the camera feed is stable. Until then, the system collects frames and updates the buffer.

```python
def obstaclesAreSimilar(obsArr1, obsArr2, tol=50.0):
  if len(obsArr1) != len(obsArr2):
    return False
  for r1, r2 in zip(obsArr1, obsArr2):
    if r1[2] != r2[2] or r1[1] != r2[1]:
      return False
    x1, y1 = round(r1[3], 1), round(r1[4], 1)
    x2, y2 = round(r2[3], 1), round(r2[4], 1)
    if abs(x1 - x2) > tol or abs(y1 - y2) > tol:
      return False
  return True
```

#### 2.7.2. Buffer design and stability

The system maintains a buffer `coordBuf` of the last N coordinate arrays. Each element is the array of with the coordinates of the corners, robot, goal, and obstacle vertices.

When a new coordinate array `coords` is available, the code compares the obstacle coords to the obstacle coords of the last buffer entry. We extract array rows where type == "vertex" and check that the same number of obstacle vertices are present, IDs and labels match, and the positions of corresponding vertices are within a our tolerance of 50 mm. If the new obstacles are within our tolerance it is appended to the buffer. If it isn't within our tolerance, the buffer is reset to contain just the new frame. Once the buffer has N similar elements, the system checks that all entries are similar to the first one:

- If yes, our feed is stable and we can return these coordinates to the demo file. 
- If no, keep getting more frames.

We are able to toggle latency and stability my modifying our hardcoded <code>bufferLen</code> values:

- Increase bufferLen:
  - Increase stability, function takes longer to get stable coordinates.
- Decrease bufferLen:
  - Decrease stability, easier to get a stable feed and will return faster.

In our program, we didn't really "tune" our bufferLen it was more just modified depending on how the obstacle detection was working. If we were waiting long times for the function to return the coordinates we would reduce the bufferLen, if the program was returning incorrect or missing coordinates we would increase the bufferLen.

---