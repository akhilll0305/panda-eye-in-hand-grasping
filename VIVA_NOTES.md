# A4: robot grasping with a camera on the hand

These notes explain the current code, help you practise viva answers, and show how to run the demo. They assume basic Python knowledge; robotics terms are explained as they appear.

Last checked: 28 October 2026. Test results and platform limits are recorded in section 10.

Use [section 2](#2-how-to-run-and-demonstrate-the-project) to run the demo, [sections 3 to 7](#3-where-the-code-is-and-how-data-moves-through-it) to understand the code, and [section 9](#9-viva-questions-with-answers-you-can-practise-aloud) to practise answers.

## 1. What the project does

A Franka Panda robot arm must find fruits on a table and put them on a plate. A camera attached to its hand provides colour images and depth images. The robot estimates object positions from those observations.

The robot knows its own joint angles. The assignment code must not ask the simulator directly where an object is. It must use `RGBDSensor.get_observation` to observe the scene. The supplied sensor helper uses simulator rendering internally; that is part of the provided setup.

The task has two parts:

- Perception: look around, build a 3D view of the scene, separate possible objects, and identify them.
- Manipulation: use the estimated positions to pick the detected fruits and place them on the plate.

The fruit classes are lemon, orange, peach, and plum. The plate is the destination. The wooden block stays in place as an obstacle. The current object dictionary supports at most one object of each type.

### A short explanation to practise aloud

> We use a camera mounted on the robot's hand to observe the table from several positions. Each colour and depth image becomes a set of 3D points. We combine the views, remove large flat surfaces, and group the remaining points into possible objects. YOLO identifies the objects in colour images. We connect those detections to the 3D groups, estimate pick and place positions, and move the fruits to the plate.

### Terms used throughout the notes

| Term | Meaning in this project |
|---|---|
| RGB image | A colour image with red, green, and blue channels. |
| Depth image | An image giving the distance along the camera's viewing axis at each pixel, in metres. |
| RGB-D | Colour and depth observations captured together. |
| Eye-in-hand | A camera attached to the robot's hand, so it moves with the arm. |
| Point cloud | A collection of 3D points describing visible surfaces. Each point can also have a colour. |
| Cluster | A group of nearby points that may belong to one object or part of an object. |
| Centroid | The average position of a set of points. |
| Bounding box | A rectangle around a detection in an image, or a box around points in 3D. |
| Joint configuration | The list of angles for the arm's seven rotating joints. |
| End effector | The working end of the robot. Here, it is the hand and gripper. |
| Pose | A position together with an orientation. |
| Coordinate frame | A reference origin and x, y, z directions used to describe positions. |
| Ground truth | The simulator's exact information, such as the true object positions. |
| Occlusion | A surface is hidden behind another object and cannot be seen from that view. |

Positions and distances in the code are in metres. Joint angles and Euler orientation angles are in radians: `pi` radians is 180 degrees.

## 2. How to run and demonstrate the project

### 2.1 Run on the Mac using the existing environment

The project now has a separate `.venv` environment. From the project folder, run:

```bash
source .venv/bin/activate
cd A4/src
python main.py
```

Run `source .venv/bin/activate` again when opening a new terminal. Once you are already in `A4/src`, run `python main.py` to repeat the demo.

The working directory matters. The starter code loads `./init_state.json`, `./assets/...`, and `./weights/best.pt` relative to the current folder. Running from `A4/src` makes those paths resolve correctly.

The Mac test environment uses Python 3.12.14, Open3D 0.19.0, Ultralytics 8.4.21, NumPy 2.0.2, PyTorch 2.8.0, and a locally built standard PyBullet 3.2.7. The course's Linux environment uses different platform builds.

PyBullet can run on Apple Silicon. Its standard package needed a build setting to avoid a conflict between its bundled older zlib code and the current Mac SDK: `CFLAGS=-Dfdopen=fdopen`. The successful build also had NumPy support enabled, which the sensor helper needs for image arrays. This setting was used while building the dependency, not in the assignment source files.

A separate package, [pybullet-arm64](https://pypi.org/project/pybullet-arm64/), also offers Mac ARM build fixes and uses the same `import pybullet` interface. The tests recorded here used standard PyBullet, not that fork. Do not install both packages into one environment because they provide the same module.

### 2.2 Run on Windows

Use the Windows Python environment that has the assignment packages installed. A Mac `.venv` cannot be copied to Windows and used there; each operating system needs its own environment.

The existing `run_demo.bat` expects this interpreter:

```text
%USERPROFILE%\micromamba\envs\a4_2026\python.exe
```

If that is where the environment is installed, double-click `run_demo.bat` in the project folder. The script changes into `A4/src`, runs the program, and pauses afterward so the terminal stays open.

To run the same environment manually, open PowerShell in the project folder:

```powershell
Set-Location .\A4\src
& "$env:USERPROFILE\micromamba\envs\a4_2026\python.exe" main.py
```

If the teammate instead has a Windows `.venv` in the project folder, use:

```powershell
Set-Location .\A4\src
& ..\..\.venv\Scripts\python.exe main.py
```

In Command Prompt, an example using micromamba is:

```bat
cd /d "D:\path\to\project\A4\src"
"%USERPROFILE%\micromamba\envs\a4_2026\python.exe" main.py
```

Replace the example project path with the actual folder. If the interpreter is installed elsewhere, use its actual path. The old notes referred to a specific laptop and drive; those paths are not shared setup requirements.

### 2.3 The course's Linux setup

The handout recommends Linux with a display. For that environment, the supplied commands are:

```bash
cd A4
conda env create -f a4_2026.yml
conda activate a4_2026
cd src
python main.py
```

The environment file contains Linux-specific package builds. It is not a portable Mac or Windows installation recipe. Keep the course environment file as supplied for evaluation, and do not add new submission dependencies just to solve a local setup problem.

### 2.4 What to watch during the demo

| Stage | What you see or read | A simple explanation to give |
|---|---|---|
| Exploration | The arm moves through its scan poses; the terminal prints `Pose 1/14`, and so on. | We are observing the table from different directions. |
| Combining views | The arm pauses while the terminal prints ICP fitness and error values. | We are aligning and combining the 3D points from those views. |
| Plane removal | The terminal reports removed planes and how many points remain. | We are removing large flat surfaces such as the floor and table. |
| Clustering | The terminal prints the groups of points it found. | We are separating possible objects without supplying an object count. |
| Identification | The arm visits viewpoints above clusters and returns home between visits. | We are connecting image detections to the groups of 3D points. |
| Planning | The terminal lists fruits and estimated pick and place positions. | We are deciding which actions to perform using the perceived scene. |
| Execution | The arm picks and places one fruit at a time. | We approach from above, grip, lift, carry, and release the fruit. |

In the tested default scene there were 14 scan poses, 8 clusters, 4 fruits, and 17 planned actions. These are results for that scene, not required counts for every run. Some clusters describe parts of the shelf rather than separate movable objects.

At normal completion, the program prints:

```text
All planned movements finished; placement success has not been checked
```

This means the action list finished. The submitted program does not look again to confirm that every fruit reached the plate. The separate test script checked final placement afterward.

The older Windows notes estimated about five minutes for an interactive run. Runtime depends on the hardware, object count, processing, and movement waits. The recent headless tests (without a display window) used a different simulation clock, so they do not measure GUI demo time.

### 2.5 Show the point clouds

In `perception.py`, find `VISUALIZE = False` near the top and change it to `True` for a local demonstration.

Open3D then shows the collected clouds, the combined cloud, the points after plane removal, and the clusters. The red, green, and blue axes mark the robot-base x, y, and z directions. Close each window to let the program continue.

For the standard Open3D viewer, drag with the left mouse button to rotate, scroll to zoom, and use Ctrl plus left-drag to move the view sideways. Press `H` for help when the window has keyboard focus. These controls are described in the [Open3D visualization guide](https://www.open3d.org/docs/0.19.0/tutorial/visualization/visualization.html).

The robot waits while these display windows are open. Restore `VISUALIZE = False` before submission or an unattended run.

### 2.6 Show the detection images

After a normal run, look in `A4/src/outputs/`. Files such as `cluster_0.png` show the YOLO boxes, class names, and confidence scores. A blue dot marks the projected cluster centre when it falls inside the image.

Use the image to explain the connection between a 3D cluster and a 2D detection. A box must contain the projected point to be considered a match. The separate automated tests saved their images outside the repository, so those runs do not populate the normal outputs folder.

### 2.7 Try a different scene

For local testing, back up `A4/src/init_state.json` before changing it. Restore the original before submission, as the handout requires.

The following scene puts the plate on the opposite side and moves the fruits. It matches the alternate scene used in the recent full pipeline test:

```json
{
  "num_objects": 6,
  "shelf_position": [0.1, -0.5, 0.05],
  "objects": {
    "lemon": [0.45, 0.2, 0.07],
    "orange": [0.6, 0.05, 0.07],
    "peach": [0.35, 0.3, 0.07],
    "plum": [0.55, 0.3, 0.08],
    "wood_block": [0.68, 0.26, 0.12]
  }
}
```

`Environment` loads objects by looping over the `objects` dictionary; it does not use `num_objects` to control that loop. Assignment logic must not use either the coordinates or that count as knowledge of the scene.

Keep local test objects on the tabletop, separated, and within the arm's reach. The older notes suggest x around 0.3 to 0.7 m, y around -0.4 to 0.4 m, and roughly 8 cm between fruit positions as a practical starting point. These are test-placement suggestions, not a guarantee that every arrangement in that area will work. Touching fruits can merge into one cluster.

For a fewer-fruit test, remove one fruit entry while keeping valid JSON. Do not create duplicate instances of a class: the current dictionary and planner are not designed for that.

### 2.8 Common problems

| What happens | What to check or do |
|---|---|
| `init_state.json`, assets, or weights cannot be found | Start the program from `A4/src`. |
| A package import fails | Check that the terminal is using the environment where the packages were installed. |
| The wrong Python starts | Run `python -c "import sys; print(sys.executable)"`, or use the full path to the correct interpreter. |
| Windows batch file cannot find Python | Check whether the micromamba path expected by `run_demo.bat` exists. Use the correct interpreter manually if it differs. |
| The arm pauses during processing | Read the terminal. ICP, plane removal, clustering, or model loading may still be working. |
| The program waits with an Open3D window open | Close that window to continue. |
| A movement times out | The code requests a hold at the current angles and stops the task. Inspect the target and possible obstruction before rerunning. |
| A target exceeds joint limits | The movement is rejected before motor commands are sent. Investigate the target rather than removing the check. |
| `Plate not found, cannot make a plan` | The plate was not identified. The planner returns an empty plan instead of inventing a destination. |
| A cluster has no matching detection | That cluster is left unlabelled. Its points may belong to background structure, or the detector may have missed the object. |

Use `Ctrl+C` in the terminal to interrupt a run, then close any remaining display windows. Before a live demo, plug in the laptop and close demanding applications so processing and simulation have enough resources.

## 3. Where the code is and how data moves through it

### 3.1 File responsibilities

| File | Responsibility |
|---|---|
| `main.py` | Calls the stages in order. It is the supplied entry point. |
| `base.py` | Supplies the environment, robot constants, finger constraint, and camera helper. |
| `control.py` | Defines `Manipulator`: joint motion, gripper commands, picking, placing, and plan execution. |
| `perception.py` | Defines image detection, camera exploration, point-cloud processing, object identification, and pose estimates. |
| `planning.py` | Builds the action list from the perceived objects. |

The assignment implementation files are `control.py`, `perception.py`, and `planning.py`. The notes are study material, not an extra runtime dependency. Follow the handout's separate requirements for the report and submission package.

### 3.2 Pipeline and data types

```text
generate_exploration_poses
  -> exlpore_scene
  -> register_pointclouds
  -> remove_planes
  -> cluster_objects
  -> identify_objects
  -> get_clear_task_plan
  -> execute_task_plan
```

| Step | Function | Output passed onward |
|---|---|---|
| 1.1 | `generate_exploration_poses` | A list of joint configurations, each containing seven angles. |
| 1.2 | `exlpore_scene` | A list of coloured Open3D point clouds, already in world coordinates. |
| 1.3 | `register_pointclouds` | One combined point cloud. |
| 1.4 | `remove_planes` | The remaining points after removing accepted large planes and cropping below the table. |
| 1.5 | `cluster_objects` | A list of point clouds, one per retained cluster. |
| 1.6 | `identify_objects` | A dictionary such as `{'lemon': lemon_cloud, 'plate': plate_cloud}`. |
| 2.1 | `get_clear_task_plan` | A list of action names and their arguments. |
| 2.2 | `execute_task_plan` | Executes the actions; an error stops later actions. |

The spelling `exlpore_scene` comes from the starter code. Keep it because `main.py` calls that exact name.

In the recent default-scene test, combining views produced 193,130 points. After plane removal, 9,409 remained, and clustering returned 8 groups. Point counts vary with the scene and the reconstruction; they are examples, not targets the code must force.

### 3.3 The classes and their relationships

```text
main.py
  Environment               loads the simulated scene
  Manipulator(PandaArm)      controls the robot
  Perception                observes and identifies objects
    RGBDSensor              supplies colour, depth, and camera settings
    DepthProcessor          works with 3D points
    ObjectDetector          loads YOLO only when identification needs it
  Planner
    its own Perception      used for pick and place pose calculations
  DepthProcessor            used directly for merging, plane removal, clustering
```

`Perception` handles observations and robot viewpoints. `DepthProcessor` handles point-cloud calculations and does not move the arm. This makes their responsibilities easy to explain separately.

The planner still owns a separate `Perception` object because of the starter structure. That object does not load another YOLO model: `Perception.__init__` sets `obj_detector` to `None`, and `identify_objects` loads it only when there are clusters to identify.

The functions inside each class are mostly arranged in the order they are used. The `STEP` labels on the main pipeline functions connect them to `main.py`. Helpers have plain descriptions rather than all repeating a step number. Search for the function name when locating an individual helper.

## 4. How the robot moves: `control.py`

### 4.1 Reading the robot's own state

`Manipulator` inherits from `PandaArm`. This gives it the supplied joint numbers, joint limits, home configuration, finger positions, and end-effector index. The base constructor also creates the gear constraint that makes the fingers move together.

`get_joint_positions()` reads the angles of the seven arm joints using `getJointStates`. Reading the robot's own state is allowed. This does not reveal where scene objects are.

`is_moving()` reads joint velocities. It returns `True` if any arm joint is moving faster than 0.01 radians per second. This helps avoid taking a picture while the arm is still settling.

### 4.2 Moving to joint angles

`move_joints(target_joints)` receives seven angles in radians.

Before moving, it checks that the input contains exactly seven valid numbers and that every angle is within the Panda's joint limits. If a target is invalid, it raises a `ValueError` before sending any motor commands.

For a valid target, it sends a position-control command to each arm joint. Position control means the motor controller tries to bring the joint to the requested angle. The arm uses a maximum motor torque of 200 and a maximum joint speed of 0.8 radians per second.

The function then waits until:

- The largest difference between a current and requested angle is below 0.01 radians.
- `is_moving()` says the arm has settled.

It checks roughly every 0.01 seconds and waits for at most 10 seconds. It uses `time.monotonic`, a clock that is not affected by adjustments to the computer's calendar clock.

If the time limit is reached, the function replaces the old motor targets with the current joint angles, asking the arm to hold there. It then raises a `TimeoutError`. The error stops later actions instead of letting the gripper close or release at an unintended position.

The speed limit helps reduce sudden motion and slipping.

### 4.3 Home position and hand targets

`move_home()` calls `move_joints` with the supplied `PANDA_HOME_POSITION`.

`move_in_task_space(target_position, target_orientation)` describes a target for the hand rather than seven joint angles:

```python
position = [x, y, z]              # metres, relative to the robot base
orientation = [roll, pitch, yaw] # radians
```

If no orientation is supplied, the default is `[pi, 0, 0]`, which points the gripper downward in this setup.

The function converts the Euler angles into a quaternion, the four-number rotation format expected by the simulator API. It then calls `calculateInverseKinematics` to find joint angles for the requested hand pose. This is inverse kinematics, or IK: position and orientation go in, joint angles come out.

The IK call supplies the joint limits, the allowed angle ranges, and the home configuration as a preferred resting shape. It allows up to 200 iterations with a residual threshold of `1e-4`. Only the first seven returned angles are sent to the arm; the remaining movable joints are the fingers.

The target link is index 11, `panda_grasptarget`, a reference point between the fingertips. This lets the target describe where the object should sit between the fingers.

IK and the subsequent joint checks do not plan a path around obstacles. Also, the control code checks whether the requested joint angles were reached; it does not separately measure and validate the final hand-position error after every IK move.

### 4.4 Opening and closing the gripper

`command_gripper(open=True)` sends each finger to 0.04 m. `open=False` sends each finger toward 0 m. The fingers are joints 9 and 10, and the command uses a maximum force of 300, followed by a one-second wait.

When closing around an object, physical contact may stop the fingers before zero. The function returns `True` to indicate that the command was sent. It does not verify a successful grasp.

The robot holds fruit through contact and friction. No attachment constraint is added between a fruit and the hand. The gear constraint only links the two fingers. The supplied fruit models have a mass of 1 kg each; their loaded lateral-friction value was checked as 0.5. These are simulation properties, not claims about real fruit.

### 4.5 Picking and placing

`pick_object_from(position, orientation)` performs a crane grasp, meaning an approach from above:

1. Open the fingers.
2. Move 20 cm above the estimated grasp position.
3. Move to 5 cm above it.
4. Move down to the grasp position.
5. Close the fingers.
6. Lift 25 cm above the grasp position.

`place_object_at(position, orientation)` moves 15 cm above the release position, lowers to it, opens the fingers, and rises 15 cm again.

These are hand waypoints. The controller sends joint targets for each one, without enforcing a straight line through space between them. Section 8 explains the movement limits.

### 4.6 Executing a plan

Each plan step contains a method name and the arguments for that method:

```python
("pick_object_from", [pick_position, pick_orientation])
```

`execute_task_plan` gets the method with `getattr(self, action)` and calls it with `function(*args)`. The `*` passes the list elements as separate function arguments.

This keeps the plan as ordinary Python data that can be printed and inspected before execution. The executor runs steps in order. A raised movement error prevents later steps from running. An empty plan prints `No actions to perform`.

## 5. How the scene is understood: `perception.py`

### 5.1 Generate exploration poses

`generate_exploration_poses()` starts with the home configuration and changes two joints:

- Base joint 0 takes seven angles from -90 to +90 degrees.
- Wrist joint 5 takes two angles, 1.49 and 1.85 radians.

The seven base angles are combined with both wrist angles, giving 14 viewpoints. Turning the base changes the viewing direction across the table; changing the wrist tilt gives a different view of the scene.

This is a fixed scan pattern rather than a view-selection algorithm. The earlier notes say camera images were inspected when choosing the poses. The pattern covered the needed objects in the three tested scenes.

The generated poses passed the joint-limit checks in those runs. The exploration loop can accept a different-length list of poses; it does not require the list to contain exactly 14.

### 5.2 Capture observations and build point clouds

`exlpore_scene(poses)` moves to each configuration and reads:

```python
rgb, depth, T_cb, intrinsics = sensor.get_observation(p, pandaId)
```

The sensor returns:

| Value | Meaning |
|---|---|
| `rgb` | A 480 by 640 colour image with RGB channel order. |
| `depth` | A 480 by 640 array of depth values in metres. |
| `T_cb` | The camera-to-base position and rotation, stored in a 4 by 4 matrix. |
| `intrinsics` | `[fx, fy, cx, cy]`, the camera's focal lengths and image centre in pixels. |

For each view, the code converts pixels into 3D points in camera coordinates, transforms those points into world coordinates, and stores the cloud. It returns home after the scan.

`rgbd_to_pointcloud` applies the pinhole-camera equations. Section 7 works through the equations and coordinate frames. It also copies the RGB colours to the points and divides colour values by 255, converting the usual 0 to 255 range into Open3D's 0 to 1 range.

The depth filter keeps only finite points with depth greater than zero and less than 1.5 m. It removes missing or invalid readings and distant background points. The 1.5 m cutoff is a processing choice; the sensor itself has a far range of 2 m.

`transform_pointcloud` copies the cloud before applying `T_cb`. This preserves the original camera-frame cloud and gives the result a common reference frame for merging.

### 5.3 Combine views with ICP

`register_pointclouds(pcds)` ignores empty clouds. If no usable clouds remain, it returns an empty Open3D cloud and prints a clear message.

All input clouds have already been transformed into world coordinates using the known camera pose. ICP, short for Iterative Closest Point, refines their alignment:

1. Find nearby matching points between the next view and the combined cloud.
2. Estimate a small rotation and translation that bring those matches closer.
3. Repeat until the library's stopping condition is reached.

Point-to-point ICP reduces the sum of squared distances between matching points. Matches must be within 2 cm. It starts from `np.eye(4)`, the identity matrix, which leaves points where they are: the camera transforms have already given an initial alignment.

Each new view is aligned to the growing combined cloud, added to it, and downsampled again. Downsampling uses a 5 mm voxel grid: imagine dividing space into small cubes and replacing points in each occupied cube with a representative averaged point.

A full 640 by 480 image has 307,200 pixels. Fourteen full views could produce over four million points before filtering. Downsampling reduces repeated nearby samples and makes later processing practical.

The terminal reports:

- Fitness: the fraction of source points that found a nearby match.
- RMSE: the root mean square distance between matched points, in metres. Smaller values mean those matches are closer.

In the recent default run, reported RMSE values were about 1.1 to 3.8 mm. A low RMSE is not proof that the entire cloud is correct: poorly overlapping views or repeated shapes can still produce a misleading alignment.

### 5.4 Remove large flat surfaces with RANSAC

`remove_plane` uses Open3D's `segment_plane`. Its plane equation is:

```text
a*x + b*y + c*z + d = 0
```

RANSAC repeatedly fits a possible plane from three sampled points and counts points close to it. Points within the distance threshold are called inliers. Points outside that distance are outliers for that candidate plane.

The code uses a distance threshold of 1 cm, a sample size of three points, and up to 1,000 iterations. The method looks for a plane supported by many points. Object points can still support an unwanted plane, so RANSAC alone does not know which surface is the table.

`remove_planes` downsamples the scene, then tries up to three plane removals. It accepts a plane only when:

- `abs(c) > 0.99`, meaning its normal is close to vertical, so the surface is close to horizontal.
- It contains more than 10,000 points, distinguishing large surfaces from smaller object surfaces in this reconstruction.

The function stops when a candidate fails those checks or fewer than three points remain. It treats the highest accepted plane as the tabletop and keeps points more than 1 cm above that height.

In the recent default run, the two removed plane heights were approximately -0.579 m for the floor and 0.045 m for the table. Those values were estimated from the cloud; the assignment logic does not use them as hard-coded object coordinates.

These thresholds are tuned for this scene scale and 5 mm downsampling. A poorly observed table or another large horizontal surface can challenge the assumptions.

### 5.5 Group nearby points with DBSCAN

`cluster_objects(scene_pcd)` returns an empty list for an empty cloud. Otherwise it uses DBSCAN, a method that forms groups based on how densely points are packed together.

The settings are `eps=0.02`, a 2 cm neighbourhood radius, and `min_points=20`:

- A core point has a neighbourhood containing at least 20 points.
- A border point is close to a core point without having enough neighbours to be a core point itself.
- Sparse points that do not belong to a cluster are noise, labelled `-1`.

DBSCAN grows clusters through connected dense neighbourhoods. It does not need a chosen object count. The code iterates over labels starting at zero, so it leaves noise points out.

After clustering, it drops groups with fewer than 100 points. It also drops clusters whose bounding-box centre is within 20 cm of the robot base in the horizontal plane. That second rule is intended to reject robot points; it would also reject a real scene object inside that region.

DBSCAN can merge touching objects or split an object with sparse observations. Increasing the radius can join nearby objects; decreasing it can break one object into smaller groups. The 2 cm radius is four times the 5 mm voxel size.

The tested default scene gave eight clusters: four fruits, the wooden block, a plate/shelf-top group, and two shelf-leg groups. A cluster is therefore an object proposal, not a guaranteed separate object.

### 5.6 Identify clusters using the colour image

`ObjectDetector.detect_objects(rgbImage)` converts RGB into BGR, the channel order expected for the NumPy image passed to this detector. It runs the supplied model at `./weights/best.pt`.

Each returned detection is:

```python
(label, [x1, y1, x2, y2], confidence)
```

The box coordinates are left, top, right, and bottom in pixels. Confidence is the model's score for the detection. The detector also returns an image with boxes and labels drawn on it.

The supplied classes are `plateId`, `orangeId`, `plumId`, `lemonId`, `peachId`, and `wood_blockId`. Removing `Id` gives the names used by the object dictionary and planner.

`identify_objects(clusters)` performs the following steps for each cluster:

1. Choose a viewpoint above it using `get_cluster_top_position`.
2. Move the hand there and capture a fresh observation.
3. Run YOLO on the colour image.
4. Project the cluster's 3D bounding-box centre into that image.
5. Find a suitable box and store the cluster under its label.
6. Return home before the next visit.

The hand target is 30 cm above the cluster's highest point, with x shifted by -4 cm to compensate for the sensor helper's offset. The camera is slightly below the hand.

The code gets the box with `get_axis_aligned_bounding_box`. Its faces follow the x, y, and z axes. Its centre is halfway between its minimum and maximum bounds. That differs from the centroid, which averages all the points.

If the cluster is behind the camera, its point cannot be projected. If the projected point is outside the image, it is not matched. Otherwise `find_closest_box_label` considers only boxes containing that point. When several boxes overlap, it chooses the box with the closest centre. The confidence score is returned by YOLO but is not used by this matching helper.

No containing box means no label. There is no fallback to an unrelated nearby box and no distance multiplier of 0.1 in the current code.

The result looks like this:

```python
objs = {
    "lemon": lemon_point_cloud,
    "plate": plate_point_cloud,
}
```

If several clusters receive the same label, the larger one is retained. This is a heuristic for the one-object-per-type scene. It helps when parts of the shelf are also associated with a plate detection. A projected point can share a pixel with a visible surface in front of it, so box matching alone does not resolve every occlusion.

This dictionary does not support two real lemons as separate instances. That would need a different representation and corresponding planner changes. Duplicate-instance support is outside the current agreed scope.

### 5.7 Estimate pick and place poses

`get_object_pick_pose(label, objs)` uses the labelled cluster:

- x and y are the average coordinates of its points.
- z is the highest point minus 3 cm, placing the grasp below the visible top.
- Orientation is `[pi, 0, 0]`, pointing the gripper down.

The reconstructed surfaces may be incomplete, so their centroid is an estimate, not necessarily the object's exact centre of mass. The 3 cm offset is a simple grasp-height choice for these fruit models, not a general rule for every object shape.

`get_obj_place_pose(object_label, target_label, objs)` uses the plate cluster's average x and y coordinates. Its release height is the plate cluster's highest point plus 8 cm. The downward orientation stays the same.

Both methods return `(position, orientation)`, with three values in each part. The planner then adds a small horizontal offset to the release position.

## 6. How the task plan is built: `planning.py`

`get_clear_task_plan(objs)` looks for the plate among the detected labels. If it cannot find one, it prints `Plate not found, cannot make a plan` and returns `[]`.

It treats labels other than plate and wood block as fruits. With this model's six classes, those are lemon, orange, peach, and plum. Only labelled objects enter the plan; an undetected fruit cannot be included just because the task goal is to clear all fruits.

The plan begins with a home move. For every fruit, `pick_place_object_plan` adds:

```text
pick_object_from -> move_home -> place_object_at -> move_home
```

For N detected fruits and a detected plate, this gives `1 + 4*N` actions: 17 for four fruits or 13 for three. If no plate is detected, the plan is empty instead.

The release offsets, in metres, are:

```python
[(0.04, 0), (-0.04, 0), (0, 0.04), (0, -0.04), (0, 0)]
```

They spread releases around the perceived plate centre to reduce stacking. The offsets cycle if there are more entries. The code does not calculate packing based on plate size or check the plate again after each placement.

Returning home between operations is a simple way to reduce risky transfers. It does not check every intermediate arm position for collisions. Planning here means choosing the action sequence and targets; it is not a full collision-aware motion planner.

## 7. The camera maths and coordinate frames

### 7.1 Why the frames matter

A pixel location alone is not a robot target. We need its depth, the camera's settings, and the camera's position and rotation relative to the robot.

The world/base frame has the robot base at its origin, with z pointing up. In this supplied scene, the robot-base and world frames coincide.

The camera frame uses the OpenCV convention: x points right in the image, y points down, and z points forward along the viewing direction. Pixel `u` is the column and `v` is the row. The image has columns 0 to 639 and rows 0 to 479.

The processing chain is:

```text
pixel + depth -> 3D camera point -> 3D world point
```

Intrinsics describe the camera's image geometry. Extrinsics describe where the camera is and how it is turned relative to another frame.

### 7.2 From a pixel to a 3D camera point

For pixel `(u, v)` with depth `Z`:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

`cx` and `cy` describe the image centre. `fx` and `fy` describe focal length in pixel units. Subtracting the centre gives the pixel's displacement from the viewing axis; scaling by depth gives its 3D displacement.

The code uses `np.meshgrid` to generate the pixel coordinates and applies these equations to the arrays together. It does not need a Python loop for every pixel.

For example, if `u=cx` and `v=cy`, then `X=0` and `Y=0`. That pixel lies on the camera's forward axis.

In the supplied helper, `fx` is about 457, `fy` about 343, and the image centre is `(320, 240)`. The focal lengths differ because the projection uses aspect 1 while the rendered image has size 640 by 480. Always use the returned intrinsics rather than substituting these example values.

### 7.3 From camera coordinates to world coordinates

`T_cb` is a 4 by 4 matrix that combines rotation and translation:

```text
T_cb = [ R  t ]
       [ 0  1 ]
```

Here `R` is a 3 by 3 rotation matrix and `t` contains the three position coordinates. Written without the extra matrix row, the operation is:

```text
point_world = R * point_camera + t
```

A homogeneous point is `[X, Y, Z, 1]`. The extra `1` lets one matrix apply both rotation and translation. These matrices can also be multiplied to combine transformations between frames.

The camera moves with the arm, so each captured image must use its own `T_cb`. Using one old camera pose for all views would put points in the wrong places.

### 7.4 From a world point back to a pixel

`project_world_to_image` reverses the process:

```text
point_camera = inverse(T_cb) * [x_world, y_world, z_world, 1]
u = fx * X / Z + cx
v = fy * Y / Z + cy
```

`inverse(T_cb)` is named `T_bc` in the code. It takes a base/world point into camera coordinates.

The helper returns `None` if the coordinates are invalid or `Z <= 0`. A zero depth would cause division by zero; a negative depth is behind the camera. The caller separately checks that the resulting pixel lies inside the image.

The dot drawn in a saved detection image is this projected cluster centre. That is how a 3D group is connected to a 2D object name.

### 7.5 What the supplied sensor helper does

`RGBDSensor` renders a 640 by 480 image with a 70-degree field of view, near distance 0.047 m, and far distance 2 m. It uses PyBullet's CPU-based TinyRenderer.

The helper adds `[0.04, 0, -0.02]` to the hand position to locate the camera. In this implementation that offset is added in world coordinates; the viewing and up directions are calculated from the hand's rotation.

The rendered depth buffer is not already a distance in metres. If its value is `d`, with near distance `N` and far distance `F`, the helper converts it using:

```text
Z = F * N / (F - (F - N) * d)
```

The helper also converts the OpenGL view convention into the OpenCV camera convention. It uses the axis-flip matrix `diag(1, -1, -1, 1)` when constructing `T_cb`.

The assignment logic should use the returned observations and camera information. It should not call direct object-pose, segmentation-mask, or camera-rendering APIs to obtain privileged scene information.

## 8. Fixes, assumptions, and possible improvements

### 8.1 What the current safeguards do

| Situation | Current behaviour |
|---|---|
| No usable scan clouds | Return an empty point cloud. |
| Fewer than three points available for plane fitting | Stop the plane-removal loop. |
| Empty cloud or only DBSCAN noise | Return no object clusters. |
| No clusters to identify | Return an empty dictionary without loading YOLO. |
| No detection box contains the projected point | Leave the cluster unlabelled. |
| Projected point is behind the camera or outside the image | Do not use it to assign a label. |
| Plate is missing from the labelled objects | Return an empty plan and explain why. |
| Joint target has invalid values or exceeds limits | Reject it before motor commands are sent. |
| Arm does not reach the target angles within 10 seconds | Request a hold at the current angles and raise an error, stopping later actions. |

These checks help the program fail clearly. They do not make every perception result or movement correct.

### 8.2 Earlier development observations

The original notes recorded the following problems and design choices. They are useful background, but the earlier trial-and-error runs were not repeated during the latest Mac verification:

- Fast arm movement reportedly threw a held fruit away. The joint-speed limit was reduced to 0.8 radians per second.
- A looser plane test reportedly removed fruit points. The earlier settings used `abs(c) > 0.9` and a plane-size rule based on 15% of the cloud. The current code uses the stricter horizontal test and a 10,000-point minimum.
- Parts of the shelf received the plate label. Keeping the larger cluster for a repeated label helped retain the plate/shelf-top group.
- Several fruits appeared in the same image. Projecting a cluster centre made it possible to choose a corresponding detection rather than simply taking the first result.
- Releasing every fruit at the same position caused stacking. The planner added small horizontal release offsets.
- The earlier Windows setup reportedly had a NumPy MKL crash and was changed to an OpenBLAS build. That is setup history, not a change to the perception or control algorithms.

The recent changes added explicit empty-input handling, stricter detection matching, movement-limit checks, timeout failures, and model loading only when needed. These behaviours were checked against the current code.

### 8.3 Limits to explain honestly in the viva

The system assumes one object per type and works best when fruits are separated. A hidden object can be missed, touching fruits can merge, and the detector can make mistakes. A list of detected labels does not prove that every real object was found.

The scan poses, grasp heights, clustering distances, and plane thresholds are fixed choices. They worked in the tested scenes, but are not a universal solution for different object sizes, clutter, or every reachable part of the table.

Returning home and using raised approach positions reduce some movement risks. There is no obstacle-aware path search or check of every intermediate arm configuration. Joint-limit checks alone do not prevent collisions.

After the initial scene estimate, pick and place actions do not use fresh camera observations to confirm their results. A slipped or missed fruit can therefore remain unnoticed by the submitted program.

The simulated sensor has idealised depth and known camera geometry. A real camera can have noise, missing depth, reflections, and calibration errors. Real robots can also have joint errors and different contact behaviour. Results in this simulator do not establish real-robot performance.

### 8.4 Improvements to discuss if asked

These are possible future ideas, not features that must be added now:

| Limitation | Possible next step |
|---|---|
| Touching fruits merge | Use image detections to help split a combined 3D group. |
| The same downward grasp orientation is used for every fruit | Estimate a useful grasp direction from the cloud. PCA, which finds the direction in which points are most spread out, is one possible tool. |
| No obstacle-aware movement planner | Use a collision model and a path-search method such as RRT, which samples possible arm configurations and checks connections between them. |
| Detector misses or mislabels objects | Improve the training data and evaluate whether fine-tuning actually helps. |
| A grasp or placement can fail unnoticed | Look again after the action and decide whether to retry. Finger spacing can provide a clue, but a nonzero gap alone is not proof of a grasp. |

Keep additions tied to an observed requirement or failure. This follows YAGNI: do not build extra machinery just because it might be useful later.

## 9. Viva questions with answers you can practise aloud

The answers here are deliberately shorter than the walkthrough. Use sections 4 to 7 when you need the calculation or code detail behind an answer.

### 9.1 Explaining the project

**What is the aim of the assignment?**

We estimate objects from an arm-mounted colour and depth camera, then use those estimates to move the fruits to a plate. The robot knows its own state, while object positions come from observations.

**What does eye-in-hand mean?**

The camera is attached to the robot's hand. Moving the arm changes the camera's viewpoint and pose, so every image needs its own camera-to-base transform.

**Why can we not use `getBasePositionAndOrientation` for objects?**

It reveals exact simulator object poses. The assignment asks us to estimate those poses from the camera, as a robot would need to do when object locations are unknown.

**Can the provided camera helper use `getCameraImage` internally?**

Yes. Rendering the simulated sensor is the helper's job. Our assignment logic uses the helper's observations rather than directly requesting camera images or segmentation masks from PyBullet.

**Why take several views?**

A single image shows only visible surfaces. Other views can reveal surfaces hidden from the first one and give a more complete point cloud. Coverage still depends on the chosen poses.

**Is the solution general or hard-coded?**

Object targets come from the observed point clouds. The planner handles different subsets of the fruit classes. The scan pattern and processing thresholds are fixed heuristics, and there is only one stored object per class. We tested three scenes, not every possible arrangement.

**What is the overall architecture?**

Sense, plan, then act. We estimate the scene, build the action list, and execute it. At the task level, it is open-loop: the camera does not recheck each manipulation result. Joint movement itself uses feedback.

### 9.2 Explaining perception

**Why do we need both colour and depth?**

The colour image helps identify what an object is. Depth supplies 3D surface positions. Combining them lets us attach an object name to points the arm can use for a target.

**What are camera intrinsics and extrinsics?**

Intrinsics describe how the camera maps 3D points to pixels. Extrinsics describe the camera's position and rotation relative to the robot base. We need both to connect an image location to a robot target.

**What is `T_cb`, and why is it a 4 by 4 matrix?**

It transforms camera coordinates into robot-base coordinates. The matrix combines rotation and translation. Using a fourth coordinate of one lets us apply both in a single multiplication.

**Why is ICP needed if the camera pose is known?**

The camera transform gives an initial alignment. ICP refines agreement between the sampled surfaces, and the assignment asks us to use it. In a real system, camera calibration or joint errors can also affect alignment.

**What does ICP need to work well?**

It needs useful overlap and a reasonable starting alignment. With too little overlap or repeated shapes, it can settle on the wrong alignment even when its reported error is small.

**What is the difference between point-to-point and point-to-plane ICP?**

Point-to-point reduces distances between matching points. Point-to-plane reduces distances along the target surface normals, meaning directions perpendicular to the surface. Point-to-plane can converge faster on suitable surfaces. We use point-to-point, which does not require estimating normals.

**Where does SVD fit into point-to-point alignment?**

SVD is a matrix calculation used to solve for a rotation that best aligns matched point sets. Open3D handles the alignment calculation; the assignment code calls its registration API.

**Why downsample with a voxel grid?**

Overlapping images produce many nearby or repeated samples. A 5 mm grid replaces points in small cubes with representative points, reducing the work in ICP, plane removal, and clustering.

**How does RANSAC find a plane? Why not fit all points at once?**

RANSAC tests planes from small random samples and looks for support from nearby points. A least-squares fit to every scene point can be pulled away from the tabletop by objects and other surfaces. RANSAC is more tolerant of those points, but still needs checks on the chosen plane.

**What RANSAC settings did we use?**

A 1 cm distance threshold, three sampled points, and up to 1,000 iterations. We remove a candidate only if it is large and nearly horizontal.

**Why DBSCAN instead of K-means?**

DBSCAN can find groups without a chosen cluster count and marks sparse points as noise. K-means requires the number of groups in advance and assigns points according to distances from cluster centres.

**What if the DBSCAN radius is too large or too small?**

Too large can join neighbouring fruits. Too small can split one fruit into pieces or leave too many points as noise. Our radius is 2 cm.

**How do we connect a YOLO box to a 3D cluster?**

We project the cluster's bounding-box centre into the image. We consider only boxes containing that pixel. If boxes overlap, we choose the closest box centre. No suitable box means no label.

**Why convert RGB into BGR?**

The sensor returns RGB. The detector's NumPy-image interface expects BGR. Swapping channels puts the colours in the expected order; leaving them swapped can affect recognition.

**What happens if two clusters get the same label?**

We keep the larger one. That helps with repeated plate labels on shelf pieces in this setup. It does not provide support for two real objects of the same class.

**What does YOLO do internally, at a high level?**

YOLOv8 is a one-stage detector: a convolutional neural network (CNN) predicts boxes and class scores in one pass through the image. The library filters overlapping duplicate detections using non-maximum suppression, or NMS. We use the supplied trained weights, not a model trained during the demo.

**Why not just use the depth inside a detection box?**

A rectangular image box can include table and background pixels as well as the object. A 3D cluster gives a separate group of surface points, though that cluster can also be incomplete or incorrectly grouped.

### 9.3 Explaining movement and planning

**What are forward and inverse kinematics?**

Forward kinematics calculates a hand pose from joint angles. Inverse kinematics finds joint angles for a requested hand pose. Our movement function uses PyBullet's IK solver.

**What are a Jacobian and damped least squares?**

A Jacobian describes how small joint-angle changes affect hand motion. Damped least squares is an IK approach that reduces unstable large updates near difficult configurations. These explain the numerical IK idea; our Python code delegates the solving to PyBullet.

**Why convert Euler angles to a quaternion?**

We describe the target as roll, pitch, and yaw because those angles are convenient to read. The simulator's IK API expects a quaternion, so the code converts the rotation format before calling it.

**Why use a crane grasp?**

Approaching from above is a simple choice for these roughly rounded fruit models. It reduces the need to estimate each object's full orientation. It still needs enough space and a useful grasp height.

**How is the grasp height chosen?**

The code uses the highest observed point minus 3 cm. That puts the target below the visible top, where the fingers can surround the fruit. It is a tuned offset, not a guarantee for every fruit size.

**How is the release position chosen?**

We use the perceived plate centre in x and y and release 8 cm above its highest observed point. Small horizontal offsets spread the release positions around the plate.

**What does `POSITION_CONTROL` mean?**

The motor controller tries to bring a joint to a requested position. For rotating arm joints, the `force` value limits motor torque; for sliding finger joints, it limits force. `maxVelocity` limits joint speed.

**How do the fingers move together?**

The supplied `PandaArm` creates a gear constraint between finger joints 9 and 10, using `gearRatio=-1`. It coordinates their motion so the fingers open and close together.

**What is link 11?**

It is `panda_grasptarget`, a reference point between the fingertips. Using it as the IK target places that reference point at the requested grasp position.

**Is the fruit attached to the robot?**

No fruit-to-hand constraint is created. Contact and friction hold the fruit. The finger gear constraint coordinates the fingers; it does not attach the object.

**Why return home between operations? Does that prevent collisions?**

It gives us a simple transfer strategy and reduces some collision risk. We do not check the whole path, so we cannot claim that returning home prevents every collision.

**What happens when the arm cannot reach its joint target?**

After 10 seconds, the code requests a hold at the current joint angles and raises a timeout error. Later plan steps do not run. Targets outside the limits are rejected before movement starts.

**What if the plate is not detected?**

The planner returns an empty plan and prints a clear message. It does not use a guessed plate position.

**Does finishing the plan prove the task succeeded?**

No. It proves the planned actions finished without a movement error. The submitted program does not confirm a grasp or recheck placement. A separate test evaluated final fruit positions.

### 9.4 Explaining the Python structure

**How would you walk through `main.py`?**

It sets up the scene and robot, generates views, captures point clouds, combines them, removes planes, forms clusters, labels them, builds a plan, and executes the actions. Section 3 shows the function names and outputs.

**Which parts are supplied and which contain the assignment implementation?**

The entry point, environment, robot constants, sensor helper, assets, initial scene, and weights are supplied. The implementation work is in `control.py`, `perception.py`, and `planning.py`. Describe each teammate's contribution from the actual work record.

**Which helpers were added beyond the main assignment functions?**

The original notes list `get_joint_positions`, `find_closest_box_label`, and the placement `offset` argument. They read joint angles, match boxes to clusters, and spread releases. Confirm the exact starter-code differences from the team's work history before discussing individual contributions.

**Why inherit from `PandaArm`?**

It provides the robot constants and setup, including joint indices, limits, home angles, finger values, and the finger constraint. `Manipulator` can use those without copying their definitions.

**Why keep the plan as method names and argument lists?**

The planner can build and print a sequence before anything moves. The controller then executes that sequence. This keeps choosing actions separate from sending motor commands.

**How does `getattr` execute the plan?**

`getattr(self, action)` looks up the method named by the action string. `function(*args)` passes the stored arguments to it. A name that does not exist raises an `AttributeError`.

**Why does the planner have its own `Perception` object? Does it load YOLO twice?**

The constructor follows the starter structure and uses that object for pose calculations. YOLO is loaded only when `identify_objects` needs it, so the planner's separate object does not load a second model.

**Why separate `Perception` and `DepthProcessor`?**

`Perception` deals with the camera, robot viewpoints, and object labels. `DepthProcessor` works on point clouds without moving the robot. This keeps those jobs easy to follow and allows point-cloud calculations to be checked separately.

**What do `VISUALIZE` and `OUTPUT_FOLDER` do?**

`VISUALIZE` enables point-cloud display windows. `OUTPUT_FOLDER` chooses where annotated detection images are saved. They help inspect the pipeline; they do not add grasp or placement verification.

### 9.5 Explaining simulation and evaluation

**How does simulation time advance?**

The normal GUI setup enables real-time simulation, and the control code waits while the arm moves. The separate headless test harness explicitly calls `stepSimulation` and supplies a simulation clock. Those are different ways of running the same pipeline.

**What camera are we simulating?**

The supplied RGB-D helper uses a 640 by 480 image, a 70-degree field of view, and near/far values of 0.047 and 2 m. Its camera position has a small offset from the hand, and the hand's rotation determines its viewing direction.

**Why can a real robot behave differently?**

Real depth can be noisy or missing, camera calibration can be imperfect, joints can have errors, and objects can slip differently. The simulator is an idealised setting, so the tested simulation results do not guarantee real-world performance.

**How was final placement checked without breaking the assignment rule?**

The submitted pipeline used camera observations for object positions. A separate evaluation script read true fruit positions afterward to score the result. Those positions did not feed into perception, target generation, or planning.

**What would you improve next?**

Choose a change based on a measured failure. Possible examples are splitting touching objects, checking a grasp with a fresh observation, or adding collision-aware movement. Explain what evidence would justify the extra complexity.

### 9.6 The tuning values in one place

| Value | Location | Meaning |
|---|---|---|
| Seven base angles, two wrist tilts | `generate_exploration_poses` | 14 views in the current scan pattern. |
| Wrist angles 1.49 and 1.85 rad | `generate_exploration_poses` | Two hand tilts for viewing the scene. |
| Valid depth between 0 and 1.5 m | `rgbd_to_pointcloud` | Reject invalid readings and distant background. |
| Voxel size 0.005 m | `DepthProcessor.__init__` | A 5 mm grid for reducing point count. |
| ICP matching distance 0.02 m | `register_pointclouds` | Matches must be within 2 cm under the current alignment. |
| Plane threshold 0.01 m; sample size 3; 1,000 iterations | `remove_plane` | RANSAC plane fitting. |
| `abs(c) > 0.99`; more than 10,000 points | `remove_planes` | Accept only large, nearly horizontal planes. |
| Up to three plane-removal attempts | `remove_planes` | A bounded loop, not automatic removal of every surface. |
| Table height + 0.01 m | `remove_planes` | Keep points more than 1 cm above the estimated table. |
| DBSCAN radius 0.02 m; 20 points | `cluster_objects` | Neighbourhood size and density requirement. |
| At least 100 points per retained cluster | `cluster_objects` | Drop very small groups. |
| At least 0.2 m horizontal distance from the base | `cluster_objects` | Reject groups close to the base. |
| Cluster top + 0.30 m; x shift -0.04 m | `get_cluster_top_position` | Hand target for an observation above a cluster. |
| Object top - 0.03 m | `get_object_pick_pose` | Estimated grasp height. |
| Plate top + 0.08 m | `get_obj_place_pose` | Release height. |
| Horizontal release shifts of 0.04 m | `get_clear_task_plan` | Spread fruits around the plate centre. |
| Pick waypoints +0.20, +0.05, and +0.25 m | `pick_object_from` | Approach and lift relative to the grasp position. |
| Place waypoints +0.15 m | `place_object_at` | Approach and retreat relative to the release position. |
| Arm speed 0.8 rad/s; torque limit 200 | `move_joints` | Limits used in arm motor commands. |
| Finger force limit 300; wait 1 second | `command_gripper` | Gripper command settings. |
| Angle error below 0.01 rad; speed at most 0.01 rad/s | Joint movement checks | Conditions for considering a move settled. |
| Wait at most 10 seconds | `move_joints` | Time limit before requesting a hold and stopping the task. |

These values serve different purposes. Know what each one controls rather than memorising them as an unexplained list.

## 10. What was tested and what the results mean

### 10.1 Full pipeline tests on the Mac

The full camera pipeline was tested on 28 October 2026 using the Mac environment listed in section 2. The runs used actual simulated camera observations, Open3D processing, the supplied YOLO weights, and the implemented planner and controller.

| Scene | Fruits placed on the plate | Full pipeline result |
|---|---|---|
| Supplied default scene | 4 of 4 | Passed. |
| Plate on the opposite side, with moved fruits | 4 of 4 | Passed. |
| Default scene with the plum removed | 3 of 3 | Passed. |
| Total | 11 of 11 | All three tested scenes passed. |

The separate harness ran PyBullet in DIRECT mode, meaning without a GUI window, and advanced physics explicitly during movement waits. The normal `main.py` still starts the provided GUI environment.

After execution, the evaluator read the true fruit positions. It checked whether each fruit centre was within the plate's horizontal area and a height range consistent with being on the plate. The horizontal test used a circle centred on the plate's bounding box, with radius equal to the smaller horizontal half-width. The height check allowed a centre between the plate's bottom and its top plus twice the fruit's current bounding-box height.

That is a geometric placement check, not a proof of contact with the plate surface. It provides a repeatable test for these scenes. The true positions were not supplied to the operational perception or planning code.

The harness also inspected contacts during movement. It found no unintended arm contacts deeper than 3 mm in these runs, excluding the fixed base and intended finger/fruit contact. This threshold-based check does not establish collision-free movement for all scenes or rule out every shallow contact.

### 10.2 Other checks

Twenty focused logic checks passed. They covered empty inputs, detection matching, projection, joint-target validation, successful movement completion, and stopping later actions after failure.

Ten additional checks used the installed libraries. They covered empty merging, tiny clouds, noise-only clustering, invalid depth filtering, empty identification, a missing plate in the planner input, and no detections from a blank image.

Those failure checks are smaller checks of specific behaviours. They are not ten additional full grasping scenes. In particular, the missing-plate check tested the planner with no plate label; it did not prove that the detector can never mistake background structure for a plate.

The dependency check also passed, and the Python files remained compatible with Python 3.9 syntax. Syntax compatibility alone does not establish compatibility with every library build in the grading environment.

### 10.3 What remains unverified

The results above are for three Mac headless scenes. The Windows GUI instructions were not retested in this verification, and neither was the official Linux grading environment. The normal Mac GUI and Open3D display interactions were also not part of these automated checks.

Do not turn 11 successful placements into a claim of a universal 100% success rate. More scenes, different object spacing, occlusion, and different platform builds can expose failures.

## 11. Older lab-progress material

The repository also has an `origin/lab-progress` branch. Earlier notes describe it as a half-finished lab-check version with an `A4_progress` folder: steps 1.1 to 1.5 work, while identification, planning, and picking/placing remain stubs. They refer to `A4_progress/run_progress.bat` and `A4_progress/PROGRESS.md` for that version.

That material is separate from the current implementation. It was not rerun as part of the latest Mac tests. If you need to inspect it, first save your current work and check `git status`; do not switch branches casually in the middle of a demo. The current full-pipeline demonstration uses the working tree described in these notes.

## 12. Keeping these notes useful

When the code changes, update the matching function description, viva answer, tuning value, and any affected demo instructions. After testing, record the date, platform, scene, and the exact success check used.

Keep observed results separate from assumptions and possible future improvements. Before describing an older development problem as something you personally encountered, confirm it against the team's actual work history.

For further reading, use the [Open3D 0.19 point-cloud guide](https://www.open3d.org/docs/0.19.0/tutorial/geometry/pointcloud.html), [ICP tutorial](https://www.open3d.org/docs/0.19.0/tutorial/pipelines/icp_registration.html), and [visualization controls](https://www.open3d.org/docs/0.19.0/tutorial/visualization/visualization.html). The code in this repository and the course handout determine what this particular implementation does.
