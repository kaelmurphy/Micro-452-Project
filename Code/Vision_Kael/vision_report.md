# Vision

## Acquisition

### 1. Overview  
The vision program is called from our demo file using a single function: getVisionCoords(timeout=, showDisplay=)

- **timeout**: number of frames to wait before timing out  
- **showDisplay**: boolean controlling whether the live camera display is shown  

---

### 2. Vision Acquisition Pipeline  
The overall workflow used for vision acquisition is as follows:

#### a) Capture camera feed  
The program initializes a camera stream and continuously reads frames for processing.

#### b) Detect ArUco markers  
We use **six ArUco markers** to represent system components:

- **IDs 0–3** → Operating zone corners  
- **ID 8** → Robot  
- **ID 9** → Goal  

These IDs are declared as constants within the program.  
(A more detailed explanation of ArUco code acquisition will be added in a later section.)

#### c) Build the operating zone  
Once the corner ArUco codes (0–3) are detected, their centers are connected to form a polygon.  
This polygon defines the **operating zone**, which is the boundary that the robot must stay within.

#### d) Compute homography  
After detecting the zone, we apply a **homography transform** (`cv2.getPerspectiveTransform`) to flatten the camera image into a top-down plane.  
This provides a coordinate system measured in **millimeters**, based on the physical dimensions we use for our zone.

#### e) Mask robot and goal for obstacle detection  
Before performing obstacle detection, the robot and goal markers are removed from the obstacle image:

- The centers of ArUco IDs **8 (robot)** and **9 (goal)** are obtained.  
- A white circle is drawn over each marker (and the robot’s shadow) to prevent them from being picked up as obstacles.  
- Obstacle detection is performed on this masked image.

#### f) Find robot orientation
We need to find the angle that the robot is at when we a querying the vision program. To do this, pass the pixel location of the center of the robot ArUco into the homography transform to return the point in mm. Then find the top left and the top right corners of the ArUco code of the robot, calculate the center point of these two using (TR - TL) / 2
where TR and TL are in pixel coordinates. Using the homography transform from earlier, we can then turn this point into world coordinates in mm. Drawing a line from the center of the ArUco code to this new point creates an angle with the horizontal axis produced by the homography transform.

#### g) Obstacle Detection
Once we have created the operating zone and gotten the angle of the robot, we check the image frame to see if there are any obstacles. The function detectObstacles(obsFrame, zone, minArea=) takes in the copy of the frame we made that has the white circles on it, the zone built for the corners, and the minimum area boundary for a valid obstacle in squared pixels. We do a lot of blurring, filtering, and masking, find the edges of the obstacles and draw contours on them. If the contours enclose an area greater than our minimum, an obstacle instance is added to the obstacle list which gets returned from this function.

#### h) Drawing
Now that we have the pixel coordinates for our zone, robot, and goal, we draw them all using the draw helpers. 

#### i) Coordinate conversion
