# Overview

- **Overall Aim:** This assignment aims to perform object-grasping tasks with a robotic arm. In this assignment, we will work in a realistic setting where the robot knows its own state but estimates objects in its workspace by collecting visual-depth observations with an RGB-D camera mounted on the robot arm.
- **Robot with an RGB-D sensor in a tabletop setting:** Consider a real-world tabletop environment with a robot arm. The robot perceives the world through an RGB-D camera mounted near its end effector (commonly referred to as an *eye-in-hand* configuration). The robot’s goal is to manipulate objects placed on a table in front of the robot. The camera moves with the robot arm and is used to estimate the state of the robot’s environment, including which objects are present, where they are located, and their extent.
- **PyBullet as a robot simulator:** We will use PyBullet to simulate the robot’s behavior. Since the assignment setting is close to the real world, access to privileged information from PyBullet is restricted. You are **permitted to use only those PyBullet functions necessary for controlling the robot and obtaining its joint states**. Functions that provide direct access to ground-truth information, such as object positions, orientations, or segmentation masks (e.g., `p.getBasePositionAndOrientation`, `p.getCameraImage`), must not be used in your implementation. Such information would **not** be available in a real-world setting. You will be provided with a helper function that returns RGB-D observations from the eye-in-hand camera, designed to more closely mimic the outputs of a real-world sensor.
- **Working with point clouds:** Point-cloud processing will be done with the [Open3D library](https://pypi.org/project/open3d/). Two capabilities need to be implemented: estimating the robot’s world in terms of objects present in the scene and grasping any object in the scene. The first part will be the majority of the assignment.
- **General comments:** To implement the pipeline, some basic stages have been provided. Once you have developed the system, you can improve the system with your own understanding and creativity.


# Environment Setup

1. **Starter code:** The starter code is available in this repository under [`A4/`](A4/). You are required to edit **only the files listed in red(control,perception,planning)** in the course handout. **Do not modify any other files.**

```text
A4
├── src
│   ├── base.py
│   ├── init_state.json
│   ├── control.py
│   ├── perception.py
│   ├── planning.py
│   ├── main.py
│   ├── weights
│   └── assets
└── a4_2026.yml
```

2. **System requirements and Conda environment:**
   1. **We strongly suggest using a Linux machine with a display for this assignment.** Using a Mac or Windows system may require a different setup, which we have not focused on. Students may need to work on their own for such alternatives.
   2. A display is required to open PyBullet in GUI mode. If you are working on a server, use X11 forwarding or run PyBullet in direct mode. For Windows and Mac, you can use external tools such as MobaXterm and XQuartz.
   3. Please install [Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and set up the environment using the environment file given below. **Do not add extra packages.** Your code will be run in a predefined Linux environment, offline. If you believe additional packages are essential, discuss them on Piazza.

```bash
# Instructions to set up on Linux
cd A4
conda env create -f a4_2026.yml
conda activate a4_2026
```

3. **PyBullet environment:** The starter code will load the simulation environment consisting of the necessary models to define the scene (ground plane, table, robot, shelf, plate, and objects). The robot arm is mounted with an RGB-D camera. Access visual and depth images through `RGBDSensor.get_observation`; do not call PyBullet’s ground-truth or camera APIs directly from assignment logic.
4. **Open3D:** We will use the Open3D library for point-cloud processing. This assignment can make use of existing functions from the library.

> **Illustration:** The initialized PyBullet environment that appears once the
> starter code is run. The original course handout provides the image.

## Repository Implementation Contract

The following conventions apply to the starter code in [`A4/src/`](A4/src/):

- Run the demo from `A4/src`, so relative paths such as `./init_state.json` and `./weights/best.pt` resolve correctly:

  ```bash
  cd A4/src
  python main.py
  ```

- `RGBDSensor.get_observation(p, panda_id)` returns `(rgb, depth, T_cb, intrinsics)`.
  The RGB image has shape `(480, 640, 3)` and RGB channel order. The depth image is
  single-channel and measured in metres. `T_cb` is a 4x4 homogeneous transform from
  camera coordinates to the robot-base/world coordinates. `intrinsics` is
  `[fx, fy, cx, cy]`, in pixels.
- The camera model uses the pinhole equations below. Pixel coordinates use `u` for
  columns and `v` for rows; depth is `Z`:

  ```text
  X = (u - cx) * Z / fx
  Y = (v - cy) * Z / fy
  Z = depth
  ```

- The expected point-cloud flow is:
  `rgbd_to_pointcloud` (camera frame) →
  `transform_pointcloud` (robot-base/world frame) →
  `register_pointclouds` →
  `remove_planes` →
  `cluster_objects` →
  `identify_objects`.
- The implemented method names and signatures are:

  ```python
  # perception.py
  Perception.generate_exploration_poses(self) -> list[list]
  Perception.exlpore_scene(self, exlporation_poses: list[list]) -> list
  Perception.identify_objects(self, pointcloud_clusters: list) -> dict
  Perception.get_object_pick_pose(self, obj_label: str, objs: dict)
  Perception.get_obj_place_pose(
      self, obj_label: str, target_obj_label: str, objs: dict
  )

  # perception.py: DepthProcessor
  DepthProcessor.rgbd_to_pointcloud(
      self, rgb, depth, intrinsics, width, height
  )
  DepthProcessor.transform_pointcloud(self, pointcloud, transform)
  DepthProcessor.register_pointclouds(self, pcds: list)
  DepthProcessor.remove_planes(self, scene_pcd)
  DepthProcessor.remove_plane(self, scene_pcd)
  DepthProcessor.cluster_objects(self, scene_pcd)

  # planning.py
  Planner.get_clear_task_plan(self, objs) -> list

  # perception.py: ObjectDetector
  ObjectDetector.detect_objects(self, rgbImage) -> tuple

  # control.py: Manipulator
  Manipulator.move_joints(self, target_joints: list)
  Manipulator.move_in_task_space(self, target_position, target_orientation=None)
  Manipulator.pick_object_from(self, obj_location: list, obj_orientation: list)
  Manipulator.place_object_at(self, target_location: list, target_orientation: list)
  Manipulator.execute_task_plan(self, plan: list)
  ```

- `Perception.exlpore_scene` is intentionally spelled `exlpore_scene` in the
  starter code and in `main.py`. Preserve that name unless you update every
  caller.
- `identify_objects` returns an `objs` dictionary mapping normalized labels
  (for example, `lemon`, `orange`, `peach`, `plum`, `plate`, or `wood_block`)
  to Open3D point clouds. The planner uses these labels to select fruits and
  the plate.
- A pick pose is `[x, y, z]` in robot-base/world coordinates plus a downward
  gripper orientation `[pi, 0, 0]`. A placement pose uses the same convention.
  Do not hard-code object coordinates: `init_state.json` is only a local test
  scene, and evaluation randomizes object positions and may change the number
  of objects.
- The current starter planner expects at most one detected cluster for each
  normalized label. If duplicate instances of the same class must be supported,
  change the `objs` representation and planner together rather than silently
  overwriting one cluster.
- `best.pt` is provided at `A4/src/weights/best.pt`. The detector receives RGB
  images but converts them to BGR internally before inference. Detection boxes
  are pixel coordinates in `[x1, y1, x2, y2]` format.
- The sensor helper necessarily uses PyBullet internally to render observations.
  Student code must use the helper and joint-state/control APIs only; it must
  not obtain object poses, segmentation masks, or camera images through direct
  ground-truth calls.

The implementation must also remain robust to evaluation randomization:

- Never use the coordinates or object count in `init_state.json` as scene
  knowledge.
- Do not assume a fixed number of exploration poses, clusters, or fruits.
- Handle empty detections and DBSCAN noise without crashing.
- Return an explicit empty result or report a clear failure when a required
  object, such as the plate, cannot be detected; do not fabricate a pose.
- Keep all generated poses within the Panda joint limits and avoid collisions
  while moving between viewpoints and manipulation waypoints.

# Problem Description

The assignment contains two parts. Part I involves development of the perception pipeline for the robot to explore the environment and estimate the objects present in the scene. The estimation is from observations captured from an RGB-D camera mounted on the robot arm. A candidate approach is presented as sub-parts in Part I which involve determining the point cloud, processing it, obtaining object proposals and recognising objects. Please follow the guidelines provided to arrive at an entire pipeline. Part II involves performing picking/placing tasks based on the objects determined in Part I. The guidelines provide an approach to performing perception. Later you may optionally improve the pipeline with your own ideas. 

The learning objective of this exercise is to gain experience with a methodology that will allow a robotic agent to operate in a general application domain where it would not be given object extents and location. Hence, keep your implementation general. In other words, you can assume that there is a table top (a planar surface) in front of the robot and the robot is aware of the *types/categories* of objects can be present. But, do not assume anything about the object locations, so that your approach works for any scene on the table top that the robot may need to interact with.  

## Part I: Estimating Objects in the Scene via Sensing

- In this part, you will develop a pipeline that allows a robot to take
  observations from the world, determine a geometric model of the interactable
  objects, and perform picking and placing tasks. The robot scans the workspace
  actively with its RGB-D camera mounted on the robot arm. Initially, the
  robot will be at the home position.
- The collected sensory observations can be fused to estimate a coherent environment representation of the scene. The point cloud would then be processed further to determine objects and their class labels.

> **Illustration:** The environment setup when the robot moves to the home
> position. The original course handout provides the image.

This exercise is divided into a sequence of steps to form a pipeline which are detailed next. Your implementation can follow the guidelines provided. Later, you can improve the pipeline with your own ideas.

### Subpart 1(a): Exploration and Recording Multiple Observations from the Arm-Mounted Camera

1. **Generation of sensing poses:** The robot must first *explore* its environment to build an understanding of the scene. Implement an approach to decide poses where the robot arm can move and the camera can acquire multiple observations of the tabletop scene. There can be several strategies for exploration. **You are free to design your own strategy.** A potential strategy for exploring an environment with multiple viewpoints can consider aspects such as:
   - If you change **yaw (left-right turning)**, the robot can explore different sides of the table. Try a 180-degree sweep.
   - If you increase the **number of poses**, the scan becomes more detailed and covers more points.
   - If you change **pitch (up-down tilt)**, the robot can see objects from different heights and angles.
    
    Specifically, please implement the following functions.
    

```python
def generate_exploration_poses(self):
    """
    Generate scanning poses.
    """
    # TODO
    pass
```

```python
def exlpore_scene(self, exlporation_poses: list[list]):
    """
    Explore the scene using generated exploration poses and return collected
    point clouds.
    """
    # TODO
    pass
```

1. **Move robot arm and record data.** Move the robot to each position to obtain an observation.

```python
def move_joints(self, target_joints):

    # TODO    
    pass
```

1. **Capture an image and depth observation (RGB-D).** A function is provided to capture an RGB-D image from the camera mounted on the robot arm. The function returns the RGB image, the aligned depth image, the camera-to-base transform, and the camera intrinsics. The image resolution is fixed by the sensor at 640 × 480 pixels.

```python
# For more details, read base.py.
rgbImg, depthImg, T_cb, intrinsics = sensor.get_observation(p, pandaId)
```

### Subpart 1(b): Point-Cloud Formation

1. **Convert RGB-D data to a point cloud in the camera frame.** Each collected RGB-D image should be converted to a point cloud (a set of points with X, Y, and Z coordinates). From each pixel in the RGB-D image `[u, v, depth]`, form a point in the camera frame using the following formula:
    
   $X = (u - cx) \times Z / fx$  
   $Y = (v - cy) \times Z / fy$  
   $Z = depth$
    
2. **Transformation to the World Frame.** Finally, transform the 3D points constituting the point cloud in the camera frame to a point cloud in the world frame. Transforming the point cloud in world frame will enable multiple point clouds of the scene to be combined together. Note that a single point cloud only represents the portion of the scene visible from that specific camera viewpoint. Hence, multiple point clouds captured from different viewpoints are to be combined together in your algorithm. Transform each resulting point from the camera coordinate system to the world coordinate frame using the known camera pose. 

```python
def rgbd_to_pointcloud(rgb, depth, intrinsics, width, height):
    """
    Convert a depth image to a 3D point cloud.
    """
    # TODO
    pass
```

> **Illustration:** RGB-D to point cloud transformed to the world frame. The
> original course handout provides the image.

1. **Combining Multiple Partial Point Clouds.** Implement a function to fuse multiple point clouds. Use the Open3D implementation of Iterative Closest Point (ICP) algorithm.  

```python
def register_pointclouds(pcds):
    """
    Merge point clouds.
    """
    # TODO
    pass
```

> **Illustration:** Example of a merged point cloud after ICP. The original
> course handout provides the image.

### Subpart 1(c): Estimating Object Clusters from a Point Cloud

Please review the method in the [Open3D library](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html).

1. **Removal of the table and ground plane from point cloud.** In order to identify the objects, first, segment out the table and the ground plane from the point cloud.
   - Plane removal from point clouds is primarily achieved using the RANSAC (Random Sample Consensus) algorithm to iteratively detect and remove the largest planar surface (walls and floors) by calculating the optimal $Ax + By + Cz + D = 0$ equation for a subset of points and filtering those within a defined threshold distance.
   - For this, you need to first load the merged point cloud and apply downsampling to speed up processing. Use *segment_plane* to identify the largest plane (ground). Set the parameters, then extract the objects by removing the identified planes.

```python
def remove_planes(self, scene_pcd):
    """
    Iteratively remove the table and other large planes.
    """
    # TODO
    pass
```

The helper used inside `remove_planes` has this signature:

```python
def remove_plane(self, scene_pcd):
    """
    Use an Open3D plane-segmentation function here.
    """
    # TODO
    pass
```

> **Illustration:** After removing the table and floor planes, typically only
> the objects remain. The original course handout provides the image.

1. **Perform clustering of the point cloud.** Use the Open3D implementation of a clustering algorithm like DBSCAN.  
   - DBSCAN (Density-Based Spatial Clustering of Applications with Noise) clusters point clouds by grouping points in high-density regions and labelling those in sparse regions as noise. Unlike K-means, it does not require you to predefine the number of clusters and can find clusters of any arbitrary shape.
   - For this, set the neighbour search radius and minimum number of points to form a cluster. For every point, find neighbours and mark it as a core, border, or noise point. Starting from an unvisited core point, expand a new cluster by recursively adding all neighbouring core points and their neighbours. Assign border points to the nearest cluster or label them as noise, then repeat until all points are processed.

```python
def cluster_objects(self, scene_pcd):
    """
    Use Open3D DBSCAN.
    """
    # TODO
    pass
```

> **Illustration:** Clustering the 3D data provides an estimate of the objects
> present. The original course handout provides the image.

### Subpart 1(d): Assigning Classification Labels to Clustered Point Clouds to Obtain Scene Objects and Their Poses

1. **Using an object detector:** Note that RGB-D data has both RGB images and a depth image. To detect objects, use an object detector on the RGB image. For this part, use YOLOv8 from Ultralytics, a real-time object-detection model that outputs bounding boxes and class labels. A trained YOLOv8 model is provided; you can fine-tune it if desired. The pre-trained weights are here:
    
   `A4/src/weights/best.pt`
    
   To use the pre-trained YOLOv8 model, import `YOLO` from `ultralytics`. The
   repository’s `ObjectDetector.detect_objects` method returns
   `(detections, annotated_image)`, where each detection is
   `(label, bbox, confidence)`.
    
    ```python
    from ultralytics import YOLO
    model = YOLO("path to the pre-trained weights")
    ```
    
    - **[Optional]** You can improve the detector with your own data training.  Each image must have a corresponding label file as .txt, which contains (class_id, x_center ,y_center, width, height). The training details are also provided. The dataset directory should be structured as follows. Model training instructions appear subsequently.

```yaml
dataset/
 ├── images/
 │    ├── train/
 │    └── val/
 ├── labels/
 │    ├── train/
 │    └── val/
 └── data.yaml
```

```python
# Define the yolo model
model = YOLO("yolov8n.pt")
# Train the YOLO model
model.train(data='./dataset/dataset.yaml', epochs=100, imgsz=640, device='cpu')
```

1. **Use the object detections to classify the corresponding 3D point cloud for an object**  

From the previous step, you will have an object detector that can provide object detections for any RGB image captured by the arm. Note that the arm can be moved over a candidate region to identify the object lying under it. 

```python
def identify_objects(self, objs_clusters):

    # TODO
    pass
```

Inside this function: 

- Implement a function to move the robot arm to move on top of the object cluster. Then capture the image *(you should convert rgbImg to bgrImg before performing detection). Note: you can go to home position after going on top of each cluster to avoid collision with other objects.*
- Next, use the object detector to process the image and obtain the bounding box proposals.  Note that each image captured may have more than one objects (especially if objects are very close to each other). Use the detections to assigning correct label to the cluster.  One way is to get cluster center, project it to the image *(using camera transform and instrincics)* and find the matching bounding box from detection. You may use the function *cluster.get_axis_aligned_bounding_box()*. Store it for all the objects in objs: dict.

*Figure: Example of a cluster for a spherical object like a circular fruit. Each point in the point cloud has XYZ and colour.*

```python
def get_cluster_top_position(self, cluster):
    # TODO
    pass
```

```python
def move_in_task_space(self, target_position):
    # TODO

    """Use move_joints to move to a joint position."""

    pass
```

*Figure: Robot moving to top of an object to capture an observation and find its semantic label*

```python
def detect_objects(self, rgbImg):
    """
    Convert the RGB image to BGR and use the trained model to assign labels.
    """
    # TODO
    pass
```

*Figure: YOLO detections from the cluster-top position.*

```text
YOLO detections: [('lemonId', array([     397.87,      372.12,      462.03,      429.86], dtype=float32)), ('orangeId', array([     456.51,      283.53,      547.15,      346.32], dtype=float32)), ('plumId', array([     356.03,      235.37,      418.39,      278.86], dtype=float32)), ('peachId', array([     309.24,      311.04,      375.04,      365.33], dtype=float32))]

```

1. **Knowing where the object is for downstream manipulation.** The above will associate the class information to the point cloud cluster. Next, perform pose estimation by obtaining the 3D pose of the target object in the scene. Use cluster centroid or max height as object pose: 

```python
def get_object_pick_pose(self, obj_label: str, objs: dict):
    # TODO
    pass

def get_obj_place_pose(
    self, obj_label: str, target_obj_label: str, objs: dict
):
    # TODO
    pass
```

Example object pose: 

```text
Object pose (world): [np.float64(0.43907262384478124), np.float64(-0.2610200090579161), np.float64(0.44650331251025654)]

```

## Part II: Object Manipulation Using the Perceived Scene

From Part I, the robot has developed an understanding of which objects are there and where are they located. In this part, we develop the pipeline to perform manipulation on the detected objects. 

1. **Manipulating an object.** Implement a function to move to the top of the object and grasp it using the crane grasping method. You will need to reason about the height at which the gripper must go above the object for appropriate picking. 

```python
def pick_object_from(self, object_location, object_orientation):
    # TODO
    pass

def place_object_at(self, target_location, target_orientation):
    # TODO
    pass
```

1. **Sequential Manipulation.** Now write a top level function that allows multiple objects to be manipulated to complete a task. The task is to put all objects in the plate. 

```python
def get_clear_task_plan(self, objs):
    # TODO
    pass

def execute_task_plan(self, plan):
    # TODO
    pass
```

# Evaluation Guidelines

The evaluation plan consists of two parts. The first part will be an offline quantitative evaluation of the pipeline (including goal reaching rate, correctness of plan and the object proposals). The second is the viva part for conceptual understanding of your work and additional efforts made by the students. The evaluation plan is approx. and may undergo minor changes.  

# Submission Instructions

## Submission Package

- Please submit the required files only. Please check and submit the correct files. Files cannot be changed after submission.
- Describe provide a brief description any innovations and new insights from your implementation. Name the file as **report.pdf** (max. 2 pages). Also, if you decide to do the assignment in pairs, then your report should **clearly mention the contribution** of each student. Both students are expected to contribute equally.
- Please create your **submission package** as follows. **Submit a single zip file** named *<A4-EntryNumber1-EntryNumber2>.zip* or *<A4-EntryNumber>.zip*. Upon unzipping, this should yield a single folder with the same name as the zip file.

## Submission Logistics

- **This assignment is to be done individually or in pairs. The choice is entirely yours.**
- The assignment is to be submitted on Google Classroom. Only **ONE** of the team members needs to submit the zip file.
- **The submission date is 11:59 pm on Sunday, November 1st, 2026. Please submit it on time.** Late submission is not allowed.
- Please strictly **adhere** to the input/output syntax specified in the assignment. Please do not modify files beyond the files you are supposed to modify. Failure to do so would cause exclusion from assessment/reduction.

