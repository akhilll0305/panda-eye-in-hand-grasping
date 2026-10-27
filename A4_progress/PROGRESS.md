# A4 progress (lab check-in)

Run: double-click `run_progress.bat` (or `cd src` then `python main.py`).
An Open3D window opens after each perception step. Close it to move on to the next step.

| main.py step | What it does | Status |
|---|---|---|
| 1.1 `generate_exploration_poses` | 14 joint poses: base yaw sweep -90..+90 deg, 2 wrist tilts | Done |
| 1.2 `exlpore_scene` | move to each pose, RGB-D -> point cloud (pinhole model) -> world frame with T_cb | Done |
| 1.3 `register_pointclouds` | merge all views, ICP to fix small misalignments, 5 mm voxel downsample | Done |
| 1.4 `remove_planes` | RANSAC removes big horizontal planes (table, floor) and anything below the table | Done |
| 1.5 `cluster_objects` | DBSCAN finds one cluster per object, drops noise and robot points | Done |
| 1.6 `identify_objects` | look at each cluster from above, YOLO, match box to cluster | Next (detector wrapper ready) |
| 2.1 `get_clear_task_plan` | plan of pick/place steps for every fruit to the plate | To do |
| 2.2 `execute_task_plan` | run the plan on the robot | To do |

Robot motion in `control.py` is also done: joint control, IK (`move_in_task_space`) and gripper open/close.
Still to do there: the pick and place sequences (crane grasp).

What you will see when you run it: the arm sweeps over the table, then the point cloud windows
(all views, merged, planes removed, coloured clusters). It stops after clustering.
