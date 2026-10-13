# A4 – Robot Grasping with Eye-in-Hand RGB-D Camera: Viva Notes

## 1. What the assignment asks (in one paragraph)

A Franka Panda arm in PyBullet has an RGB-D camera on its hand ("eye-in-hand"). The robot knows its own
joint angles, but it **does not know where the objects are**. We are **not allowed** to use
`p.getBasePositionAndOrientation` or `p.getCameraImage` directly. Only the provided sensor
(`RGBDSensor.get_observation`) may be used. The robot must:

* **Part I (perception):** look around, build a 3D point cloud of the table, remove the table/floor,
  split the remaining points into objects, and label each object with YOLOv8.
* **Part II (manipulation):** pick every fruit (lemon, orange, peach, plum) and put it on the plate,
  which sits on a small shelf. The wooden block is an obstacle that is not moved.

Only `control.py`, `perception.py` and `planning.py` were edited (the files marked red in the assignment).

## 2. How to run and demo on my laptop

Everything is already installed on this laptop. The Python environment is at
`C:\Users\Akhil Reddy\micromamba\envs\a4_2026`. A full run takes about 5 minutes.

### Option A: easiest (double-click)
1. Open the folder `D:\My Stuff\sem5\AI For Autonomous Systems\Final_Assignment` in File Explorer.
2. Double-click **`run_demo.bat`**.
3. A black terminal window opens and shows the progress text, then the **PyBullet window** opens with the
   robot, table, fruits, wooden block and plate on the shelf.
4. Don't click inside the PyBullet window while it runs. Watch it happen (see "What the examiner will
   see" below).
5. When the terminal prints `Task plan finished`, it is done. Press any key in the terminal to close it.

### Option B: from a terminal (looks more professional in a viva)
**PowerShell** (the default terminal in VS Code and Windows Terminal):
```
cd "D:\My Stuff\sem5\AI For Autonomous Systems\Final_Assignment\A4\src"
& "$env:USERPROFILE\micromamba\envs\a4_2026\python.exe" main.py
```
**Command Prompt (cmd)**:
```
cd /d "D:\My Stuff\sem5\AI For Autonomous Systems\Final_Assignment\A4\src"
"%USERPROFILE%\micromamba\envs\a4_2026\python.exe" main.py
```
Always run from inside `A4\src`, because the code loads `./init_state.json`, `./assets/...` and
`./weights/best.pt` with relative paths. Running from another folder gives "file not found".

### What the examiner will see (and what to say)
| On screen | Terminal prints | Say |
|---|---|---|
| Arm sweeps left→right twice, pausing at each pose | `Pose 1/14 ... Pose 14/14` | "Exploration: 14 viewpoints, 180° yaw sweep at 2 wrist tilts, one RGB-D image each" |
| Arm goes home, pauses a few seconds | `ICP view 1: fitness..., rmse...` | "Merging the 14 point clouds; ICP refines alignment" |
| (no motion) | `Removed plane 1: z=-0.579` / `Removed plane 2: z=0.045` | "RANSAC removed the floor and the table top" |
| (no motion) | `Cluster 0..7 ... Found 8 object clusters` | "DBSCAN found 8 groups of points" |
| Arm goes above each object, then back home | `Cluster 0: lemon ...` etc. | "Camera looks down at each cluster; YOLO labels it" |
| (no motion) | `Objects to put in plate: [...]`, pick/place poses | "Planner made the plan from the perceived positions" |
| Arm picks each fruit from above, carries it to the plate | `Step 1/17 ... Step 17/17` | "Crane grasp: down from above, close, lift, place in plate" |

### Show the point clouds (optional, very good for the viva)
1. Open `A4\src\perception.py` and change line 11 (`VISUALIZE = False`, near the top) to `VISUALIZE = True`.
2. Run as above. An **Open3D window** appears at 4 stages: the 14 raw clouds → the merged cloud → after
   plane removal (only objects left) → the clusters in random colours.
3. In the Open3D window: **left-drag** rotates, **scroll** zooms, **right-drag / ctrl+drag** moves.
   The red/green/blue axes are the robot base frame (x/y/z). **Close the window** to let the
   program continue. The robot waits while the window is open.
4. **Set it back to `VISUALIZE = False`** before submitting.

### Show the YOLO detections
After a run, open `A4\src\outputs\`. `cluster_0.png`, `cluster_1.png`, ... are the images taken above
each cluster, with YOLO boxes, labels and confidence. The **blue dot** is where that cluster's 3D centre
projects in the image; the box containing it gives the cluster's label.

### Show it works on a different scene (proves it is not hard-coded)
1. Make a copy of `A4\src\init_state.json` as a backup (e.g. `init_state_original.json`).
2. Edit `init_state.json` and change positions, for example put the plate on the other side:
   ```json
   {"num_objects": 5, "shelf_position": [0.1, -0.5, 0.05],
    "objects": {"lemon": [0.45, 0.2, 0.07], "orange": [0.6, 0.05, 0.07],
                "peach": [0.35, 0.3, 0.07], "plum": [0.55, 0.3, 0.08],
                "wood_block": [0.65, -0.25, 0.12]}}
   ```
   (This scene was tested: 4/4.) Keep objects on the table (x ≈ 0.3–0.7, y ≈ −0.4–0.4), at least
   ~8 cm apart, and not on top of each other.
3. Run again. **Restore the original file before submitting.**

### If something goes wrong
* **"No such file or directory: ./init_state.json" / weights not found:** you are not in `A4\src`.
  `cd` there first.
* **"python is not recognized" or the wrong Python:** use the full path to the environment's
  `python.exe` exactly as above (plain `python` is a different Python 3.13 without these packages).
* **The window seems frozen for a few seconds:** this is normal during ICP/RANSAC/DBSCAN and YOLO
  loading. Watch the terminal text.
* **The program waits forever with `VISUALIZE = True`:** close the Open3D window.
* **To stop a run:** close the PyBullet window or press `Ctrl+C` in the terminal.
* **Before the viva:** close heavy apps and plug in the charger; the simulation runs in real time, so a
  slow laptop makes the arm slower.

### On the lab's Linux machine (official setup)
If the demo must run on a Linux lab machine, use the course environment instead:
  ```
  cd A4
  conda env create -f a4_2026.yml
  conda activate a4_2026
  cd src
  python main.py
  ```
  `main.py` must be run from inside `src`, because the asset and weight paths are relative.

Results measured on 3 different scenes: **11/11 fruits placed on the plate**
(default scene 4/4, plate on opposite side 4/4, a 3-fruit scene 3/3).
*How this was measured:* a separate test script (not part of the submission) ran the same pipeline. Only
**after** the task finished did it read the true fruit positions from PyBullet to check whether each fruit
was on the plate. The submitted code never uses ground truth.

## 3. The pipeline (the order in main.py)

```
generate_exploration_poses → exlpore_scene → register_pointclouds → remove_planes
→ cluster_objects → identify_objects → get_clear_task_plan → execute_task_plan
```

| Step | Function | What it does | Output |
|---|---|---|---|
| 1.1 | `generate_exploration_poses` | 14 joint configurations around the table | list of 7-joint lists |
| 1.2 | `exlpore_scene` | move to each pose, take RGB-D, make point cloud in world frame | 14 point clouds |
| 1.3 | `register_pointclouds` | refine alignment with ICP and merge | 1 point cloud (~190k pts) |
| 1.4 | `remove_planes` | RANSAC removes floor and table | ~9.5k object points |
| 1.5 | `cluster_objects` | DBSCAN splits points into objects | list of clusters |
| 1.6 | `identify_objects` | camera goes above each cluster, YOLO labels it | `{'lemon': cluster, ...}` |
| 2.1 | `get_clear_task_plan` | list of actions: pick fruit → home → place on plate → home | plan list |
| 2.2 | `execute_task_plan` | calls each Manipulator function in the plan | robot moves |

## 4. control.py: Manipulator (robot motion)

* **`get_joint_positions()`** (helper I added): reads the 7 joint angles with `p.getJointStates`. This is
  the robot's own state, so it is allowed.
* **`is_moving()`**: True if any joint velocity is above 0.01 rad/s.
* **`move_joints(target)`**: sends a position-control command to each of the 7 joints with
  `setJointMotorControl2` (force 200, **maxVelocity 0.8 rad/s**), then waits in a loop until every joint
  error is < 0.01 rad and the robot has stopped (10 s timeout).
  *Why maxVelocity?* Without it the arm swung so fast that the grasped 1 kg fruit was thrown away
  (we observed this in testing: the lemon landed back on the table). Slower motion fixed it.
* **`move_home()`**: `move_joints(PANDA_HOME_POSITION)`.
* **`move_in_task_space(position, orientation)`**: Cartesian target → joints using **inverse kinematics**
  (`p.calculateInverseKinematics` on link 11 = the grasp point between the fingers). Euler angles are
  converted to a quaternion first. Joint limits and the home pose as "rest pose" keep the solution
  close to a natural arm shape. Only the first 7 values are used (the others are the fingers).
  Default orientation `[π, 0, 0]` means the gripper points **straight down**.
* **`command_gripper(open)`**: both finger joints (9, 10) go to 0.04 m (open) or 0 (closed) with
  force 300, then wait 1 s. When closing on a fruit the fingers stop at the fruit's width (~0.02 each).
* **`pick_object_from(location, orientation)`**: the **crane grasp**: open → go 20 cm above → 5 cm
  above → down to the grasp point → close → lift 25 cm. Coming straight down from above avoids
  knocking neighbouring objects sideways.
* **`place_object_at(location, orientation)`**: go 15 cm above the target → go down → open → go up.
* **`execute_task_plan(plan)`**: each step is `(function_name, args)`. `getattr(self, name)(*args)`
  calls the function, so the plan is plain data and the robot just executes it.

## 5. perception.py

### ObjectDetector.detect_objects(rgb)
1. RGB → BGR with `cv2.cvtColor` (OpenCV/YOLO expects BGR; the assignment asks for this explicitly).
2. Run YOLOv8 (`weights/best.pt`). Classes: `plateId, orangeId, plumId, lemonId, peachId, wood_blockId`.
3. Return a list of `(label, [x1,y1,x2,y2], confidence)` plus an annotated image.

### generate_exploration_poses()
Start from the home pose and change only:
* **joint 0 (base yaw)**: 7 values from −90° to +90° (a 180° sweep, as the assignment suggests), so the
  camera looks at the left, centre and right of the table.
* **joint 5 (wrist pitch)**: 1.49 (home: camera looks at the table near the robot) and 1.85 (camera
  tilted to look further out).

7 × 2 = **14 poses**. More poses give more coverage but take more time. Changing pitch shows objects
from different angles, so the back sides of fruits are also seen. I picked these by saving camera images
from candidate poses and checking that all objects were visible.

### exlpore_scene(poses)
For each pose: `move_joints` → `sensor.get_observation()` (returns `rgb, depth, T_cb, intrinsics`) →
`rgbd_to_pointcloud` → `transform_pointcloud(pcd, T_cb)`. Returns the list of world-frame clouds.

### DepthProcessor.rgbd_to_pointcloud(rgb, depth, intrinsics)
Pinhole camera model, applied to every pixel (u = column, v = row):
```
Z = depth
X = (u − cx) · Z / fx
Y = (v − cy) · Z / fy
```
Done with `np.meshgrid` for all 640×480 pixels at once (no slow for-loop). Points farther than 1.5 m
(background) are dropped. Colours are RGB/255 because Open3D wants values in 0–1.
Intrinsics here: fx ≈ 457, fy ≈ 343, cx = 320, cy = 240. fx ≠ fy because the sensor renders a 640×480
image with aspect = 1.

### transform_pointcloud(pcd, T)
`T_cb` is the 4×4 homogeneous transform of the **camera frame w.r.t. the robot base (world) frame**.
`p_world = R · p_cam + t`, done by `pcd.transform(T)`. After this, all 14 clouds share one frame and
can be combined. The robot base is at the origin, so base frame = world frame.

### register_pointclouds(pcds)
* Since we already know the camera pose, all clouds are *almost* aligned. **ICP (Iterative Closest
  Point)** fixes small leftover errors.
* ICP repeats two steps: (1) for each source point, find the closest target point (within
  `max_correspondence_distance` = 2 cm); (2) compute the rotation and translation that minimise the
  squared distance between these pairs (point-to-point, solved with SVD). This continues until it converges.
* Starts from the identity (`init = np.eye(4)`) because the clouds are already in the world frame.
* Each new cloud is aligned to the growing merged cloud, added, then **voxel-downsampled** (5 mm), so the
  size stays manageable (14 × 307k points → ~190k).
* The log prints **fitness** (fraction of points that found a partner) and **RMSE** (≈2–4 mm, so views
  were already well aligned).

### remove_plane / remove_planes
* **RANSAC** for a plane: pick 3 random points → plane `ax+by+cz+d=0` → count points within
  `distance_threshold` = 1 cm (inliers). Repeat 1000 times and keep the plane with the most inliers.
  It is robust because outliers (objects) never get to vote for a "wrong" plane.
* `remove_planes` loops up to 3 times. A found plane is removed **only if** it is horizontal
  (|c| > 0.99, normal points up) **and** big (> 10 000 points). In the default scene it removes the
  floor (z = −0.58) and then the table top (z = 0.045).
* Finally, everything below table height + 1 cm is cropped (table edges and legs are not objects).
* *Problem faced:* first I used |c| > 0.9 and "15 % of points". RANSAC then found a **tilted plane**
  through the tops of fruits and the shelf and deleted the fruits. Making the test strict fixed it.

### cluster_objects(scene_pcd)
* **DBSCAN** with `eps = 0.02` (2 cm neighbour radius) and `min_points = 20`.
  * Core point: ≥ min_points neighbours within eps. Border point: near a core point but not core itself.
    Noise: neither (label −1).
  * A cluster grows from a core point through all reachable core points and their neighbours.
  * You **do not need to give the number of clusters** (unlike K-means), and clusters can have any shape.
* Clusters with < 100 points are dropped (noise), and so is anything within 20 cm of the robot base in xy
  (points on the robot itself).
* Default scene gives 8 clusters: 4 fruits, wood block, plate + shelf top, and 2 shelf legs.
  eps = 2 cm is small enough to keep fruits that are about 4–5 cm apart separate.

### identify_objects(clusters)
For each cluster:
1. `get_cluster_top_position`: centre of the bounding box, 30 cm above its top. x is shifted by −4 cm
   because the camera is mounted 4 cm in front of the end effector (see `base.py`).
2. `move_in_task_space` there (gripper/camera facing down) and take an image.
3. Run YOLO.
4. **Project the cluster centre into the image** with `project_world_to_image`, then choose the box
   that contains that pixel (or the nearest box). This matters because one image can show several
   fruits.
5. Store `objs[name] = cluster`. If a label appears twice (e.g. a shelf leg seen as "plate"), keep the
   **bigger** cluster, since each object type appears only once.
6. Go back home before the next cluster (avoids hitting things).

### project_world_to_image(point, intrinsics, T_cb)
The reverse of the point-cloud formula: `p_cam = inv(T_cb) · p_world`, then
`u = fx·X/Z + cx`, `v = fy·Y/Z + cy`.

### get_object_pick_pose / get_obj_place_pose
* **Pick:** x, y = centroid of the cluster; z = **top of object − 3 cm**, so the fingers close around
  the upper middle of the fruit. Orientation `[π,0,0]` (top-down crane grasp).
* **Place:** x, y = centroid of the plate cluster; z = top of plate + 8 cm, so the fruit is released just
  above the plate without the fingers touching it.

## 6. planning.py: Planner

* `get_clear_task_plan(objs)`: find the plate label, make the list of fruits (everything that is not
  plate or wood_block), and add a pick-and-place plan for each fruit. Each fruit gets a small offset
  (±4 cm) on the plate so fruits do not land on top of each other.
* `pick_place_object_plan(obj, target, objs, offset)`: returns
  `[pick_object_from, move_home, place_object_at, move_home]` with poses taken from perception.
* The plan is **just a list of (function name, arguments)**. Planning (deciding what to do) is kept
  separate from control (how to move), which makes it easy to print, check, and debug.

## 7. Coordinate frames (common viva topic)

* **World / base frame:** robot base at the origin, z up. The table top is at z ≈ 0.045.
* **Camera frame (OpenCV convention):** x right, y down, z forward (along the view direction).
  `base.py` builds `T_cb` from PyBullet's view matrix (OpenGL convention: y up, z backwards) and
  multiplies by `diag(1,−1,−1,1)` to flip to the OpenCV convention.
* **Image:** u = column (0–639), v = row (0–479).
* **Chain:** pixel + depth → camera frame 3D (intrinsics) → world 3D (extrinsics `T_cb`).

## 8. Problems faced and fixes (good to mention as "insights")

1. **The arm threw the fruit while moving home:** the joints moved too fast. Fixed with
   `maxVelocity=0.8` in `move_joints`.
2. **RANSAC removed the fruits:** a tilted plane through the object tops was accepted. Fixed with a
   strict horizontal check (|c| > 0.99) and a minimum plane size.
3. **The shelf splits into several clusters** (thin legs): the legs also get the "plate" label. Fixed by
   keeping only the biggest cluster for each label.
4. **Several fruits in one image:** solved by projecting the cluster centre into the image and matching
   it to the right bounding box.
5. **Fruits stacking on the plate:** a different small offset for each fruit.
6. (Setup) The conda MKL build of numpy crashed on Windows, so I switched to OpenBLAS. This is not part of
   the code.

## 9. Likely viva questions and short answers

**Q: Why can't we use `getBasePositionAndOrientation`?**
It gives ground-truth object poses, which a real robot never has. We must estimate everything from the
camera, as in the real world.

**Q: What is eye-in-hand?**
The camera is mounted on the robot's hand, so it moves with the arm. We can choose viewpoints, but the
camera pose changes, so every image needs its own `T_cb`.

**Q: Why multiple viewpoints?**
One view sees only the front surfaces and part of the table (occlusion, limited field of view). Combining
views gives a more complete 3D model.

**Q: If you already know the camera pose, why ICP?**
Real poses have small errors (joint encoder noise, calibration). ICP refines the alignment. In our
simulation the clouds were already close (RMSE ~3 mm), so ICP makes only small corrections.

**Q: What does ICP need to work?**
A good initial guess (the clouds must already be roughly aligned) and enough overlap. Otherwise it can
converge to a wrong local minimum.

**Q: Point-to-point vs point-to-plane ICP?**
Point-to-point minimises distances between matched points. Point-to-plane minimises the distance
along the target's surface normal and usually converges faster on flat surfaces, but needs normals. I
used point-to-point because it is simpler.

**Q: Why voxel downsampling?**
14 × 307k points is too many. A 5 mm voxel grid keeps one averaged point per cube, which removes
duplicate points from overlapping views and makes RANSAC, DBSCAN and ICP much faster.

**Q: Explain RANSAC. Why not least squares?**
Least squares on all points would be pulled by the objects (outliers). RANSAC fits a plane to random
minimal samples (3 points) and keeps the plane with the most inliers, so outliers are ignored.

**Q: RANSAC parameters?**
`distance_threshold = 0.01`: how close a point must be to count as on the plane (1 cm, larger than the
noise but smaller than the fruits). `ransac_n = 3`: points needed to define a plane.
`num_iterations = 1000`: number of trials; more trials give a higher chance of finding the best plane.

**Q: Why DBSCAN and not K-means?**
K-means needs the number of clusters in advance (unknown) and gives round clusters. DBSCAN finds the
number automatically, handles any shape, and labels isolated points as noise.

**Q: What happens if eps is too big or too small?**
Too big: nearby fruits merge into one cluster. Too small: one object breaks into many pieces or becomes
noise. 2 cm is about 4 voxel sizes and smaller than the gaps between fruits.

**Q: How do you match a YOLO box to a 3D cluster?**
Project the cluster's 3D centre into the image with the intrinsics and the inverse camera pose, then pick
the box containing that pixel.

**Q: Why convert RGB to BGR?**
OpenCV and Ultralytics treat a numpy image as BGR. The sensor gives RGB, so without converting, red and
blue are swapped and detection gets worse (an orange would look blue).

**Q: What is inverse kinematics?**
Given a desired end-effector pose, find joint angles that achieve it. PyBullet solves it numerically
(iterative, damped least squares using the Jacobian). Forward kinematics is the opposite: joints → pose.

**Q: Why the crane grasp?**
Approaching vertically from above is simple, needs no object orientation, and the fruits are roughly
round. The fingers come down without pushing neighbouring objects.

**Q: How did you decide the grasp height?**
Top of the cluster − 3 cm, which is roughly the upper middle of a ~6 cm fruit. The fingers close on the
widest part, and the fingertips stay above the table.

**Q: How is your solution general and not hard-coded?**
Nothing uses object positions from the JSON file. The exploration poses cover the whole table, and all
positions come from the point cloud. The planner works for any number of fruits and any plate
location. It was tested on 3 different scenes.

**Q: What would fail / what could you improve?**
* Fruits touching each other would merge into one DBSCAN cluster. One fix is to split it using YOLO boxes.
* A fixed top-down grasp yaw. PCA on the cluster could give the long axis of the lemon to align the
  fingers with.
* No collision-aware path planning; it relies on going home between moves. A real planner (RRT)
  would be better.
* Fine-tune YOLO on more images (the assignment's optional part).
* Close the loop: check after picking whether the object is really in the gripper (e.g., finger
  width > 0), and retry if not.

**Q: What does T_cb contain?**
A 4×4 matrix: top-left 3×3 = rotation of the camera axes in the base frame, top-right 3×1 = camera
position in the base frame, bottom row [0 0 0 1].

**Q: How are depth values computed in base.py?**
The PyBullet depth buffer is non-linear (0–1). `base.py` converts it to metric depth with
`far·near / (far − (far − near)·d)`.
