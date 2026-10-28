"""Find objects from colour and depth images, then estimate where to pick and place."""

from base import RGBDSensor
from control import Manipulator

from ultralytics import YOLO
import open3d as o3d
import numpy as np
import cv2
import os

# set this to True to see the pointcloud after every step (useful for demo)
VISUALIZE = False
# detection images from identify_objects are saved in this folder
OUTPUT_FOLDER = "./outputs"

# Inside each class the functions are written in the order they are used in main.py.
# "STEP x.y" in a docstring is the step number from main.py.


class ObjectDetector:
    """Use the supplied YOLO model to find labelled boxes in a camera image."""
    def __init__(self,):
        self.detector = YOLO("./weights/best.pt")

    def detect_objects(self, rgbImage):
        """Return detections and an image with their boxes drawn on it.

        Each detection is (label, [left, top, right, bottom], confidence).
        The sensor supplies RGB colours; the model expects BGR, so swap the channels.
        """
        bgrImage = cv2.cvtColor(rgbImage, cv2.COLOR_RGB2BGR)
        result = self.detector(bgrImage, verbose=False)[0]

        detections = []
        for box in result.boxes:
            class_id = int(box.cls[0])
            label = result.names[class_id]          # e.g. 'lemonId'
            bbox = box.xyxy[0].cpu().numpy()        # [x1, y1, x2, y2] in pixels
            confidence = float(box.conf[0])
            detections.append((label, bbox, confidence))

        # image with boxes drawn on it (only used for saving / showing)
        annotated_image = result.plot()
        return detections, annotated_image


class Perception:
    """Use camera observations to find object labels and positions for the planner."""
    def __init__(self, manipulator:Manipulator):
        self.sensor = RGBDSensor()
        # Planning only needs pose calculations. Load YOLO later, when an image needs labelling.
        self.obj_detector = None
        self.manipulator = manipulator
        self.depth_processor = DepthProcessor()

    def get_observation(self,):
        """Read colour, depth, camera position and camera settings from the sensor helper."""
        p = self.manipulator.pybullet_client
        return self.sensor.get_observation(p, self.manipulator.pandaId)

    def generate_exploration_poses(self,):
        """STEP 1.1: Choose joint angles for looking across the table from several views."""
        # Idea: start from home pose and only rotate the base joint (yaw)
        # from -90 deg to +90 deg (180 deg sweep) so camera looks at all sides of table.
        # We do this sweep twice with 2 different wrist tilts (pitch):
        #   tilt 1 (home wrist angle)   -> camera looks at table near the robot
        #   tilt 2 (wrist bent forward) -> camera looks further away on the table
        yaw_angles = np.linspace(-np.pi / 2, np.pi / 2, 7)
        wrist_angles = [1.49, 1.85]

        poses = []
        for wrist in wrist_angles:
            for yaw in yaw_angles:
                pose = list(self.manipulator.PANDA_HOME_POSITION)
                pose[0] = yaw      # joint 0 rotates the whole arm left / right
                pose[5] = wrist    # joint 5 tilts the hand (camera) up / down
                poses.append(pose)
        print(f"Generated {len(poses)} exploration poses")
        return poses

    def exlpore_scene(self, exlporation_poses:list[list]):
        """STEP 1.2: Visit each pose and return its coloured 3D points in world coordinates.

        Each pose contains seven joint angles. Preserve the starter's spelling
        of this method because main.py calls it by this name.
        """
        pointclouds = []
        for i, pose in enumerate(exlporation_poses):
            self.manipulator.move_joints(pose)
            rgb, depth, T_cb, intrinsics = self.get_observation()

            # pointcloud in camera frame -> pointcloud in world (robot base) frame
            pcd_camera = self.depth_processor.rgbd_to_pointcloud(rgb, depth, intrinsics,
                                                                 self.sensor.IMG_WIDTH, self.sensor.IMG_HEIGHT)
            pcd_world = self.depth_processor.transform_pointcloud(pcd_camera, T_cb)
            pointclouds.append(pcd_world)
            print(f"Pose {i + 1}/{len(exlporation_poses)}: {len(pcd_world.points)} points")

        self.manipulator.move_home()
        if VISUALIZE:
            self.depth_processor.visualize_pointcloud(pointclouds)
        return pointclouds

    def identify_objects(self, pointcloud_clusters):
        """STEP 1.6: Look above each group of points and match it to a detected object.

        Return a dictionary such as {'lemon': lemon_point_cloud}. Groups without
        a matching image detection stay unlabelled and are left out of the plan.
        """
        if not pointcloud_clusters:
            print("No object clusters available to identify")
            return {}
        if self.obj_detector is None:
            self.obj_detector = ObjectDetector()
        os.makedirs(OUTPUT_FOLDER, exist_ok=True)
        objs = {}
        for i, cluster in enumerate(pointcloud_clusters):
            # 1. move the camera on top of the cluster and take a picture
            top_position = self.depth_processor.get_cluster_top_position(cluster)
            self.manipulator.move_in_task_space(top_position)
            rgb, depth, T_cb, intrinsics = self.get_observation()

            # 2. run YOLO on the picture
            detections, annotated_image = self.obj_detector.detect_objects(rgb)

            # 3. Find where the 3D cluster centre appears in this camera image.
            center = cluster.get_axis_aligned_bounding_box().get_center()
            pixel = self.depth_processor.project_world_to_image(center, intrinsics, T_cb)
            best_label = None

            # Only match and draw points that are visible in front of the camera.
            if pixel is not None:
                u, v = pixel
                height, width = rgb.shape[:2]
                if 0 <= u < width and 0 <= v < height:
                    best_label = self.find_closest_box_label(u, v, detections)
                    cv2.circle(annotated_image, (int(u), int(v)), 6, (255, 0, 0), -1)
            cv2.imwrite(f"{OUTPUT_FOLDER}/cluster_{i}.png", annotated_image)

            # 4. store the cluster under its label
            if best_label is None:
                print(f"Cluster {i}: no matching detection, leaving it unlabelled")
            else:
                name = best_label.replace("Id", "")   # 'lemonId' -> 'lemon'
                print(f"Cluster {i}: {name}  (center {np.round(center, 3)})")
                # every object type is present only once, so if a label is seen twice
                # (e.g. shelf leg next to plate) we keep the bigger cluster
                if name in objs and len(objs[name].points) > len(cluster.points):
                    print(f"   keeping the bigger cluster already stored for {name}")
                else:
                    objs[name] = cluster

            # Returning home reduces collision risk; it does not check the path for obstacles.
            self.manipulator.move_home()
        return objs

    def find_closest_box_label(self, u, v, detections):
        '''
        Match a cluster's image position to a detection box containing that point.
        If boxes overlap, choose the one with the closest centre. Return None
        when no box contains the point, rather than guessing a nearby object's label.
        '''
        best_label = None
        best_distance = float("inf")
        if not np.isfinite([u, v]).all():
            return None
        for label, bbox, confidence in detections:
            x1, y1, x2, y2 = bbox
            if not (x1 <= u <= x2 and y1 <= v <= y2):
                continue
            box_center_u = (x1 + x2) / 2
            box_center_v = (y1 + y2) / 2
            distance = np.sqrt((u - box_center_u) ** 2 + (v - box_center_v) ** 2)
            if distance < best_distance:
                best_distance = distance
                best_label = label
        return best_label

    def get_object_pick_pose(self, obj_label:str, objs:dict):
        """Return the fruit's grasp position in metres and downward orientation in radians."""
        points = np.asarray(objs[obj_label].points)
        center = points.mean(axis=0)            # centroid of the cluster
        max_height = points[:, 2].max()         # top of the object
        # Grasp 3 cm below the visible top so the fingers surround the fruit.
        grasp_height = max_height - 0.03
        position = [center[0], center[1], grasp_height]
        orientation = [np.pi, 0, 0]             # gripper facing down (crane grasp)
        print(f"Pick pose of {obj_label}: {np.round(position, 3)}")
        return position, orientation

    def get_obj_place_pose(self, obj_label:str, target_obj_label:str, objs:dict):
        """Return a release position above the sensed plate and a downward orientation.

        Position is [x, y, z] in metres; orientation is [roll, pitch, yaw] in radians.
        """
        points = np.asarray(objs[target_obj_label].points)
        center = points.mean(axis=0)
        max_height = points[:, 2].max()
        # release the object a bit above the plate so the fingers do not hit the plate
        position = [center[0], center[1], max_height + 0.08]
        orientation = [np.pi, 0, 0]
        print(f"Place pose for {obj_label} on {target_obj_label}: {np.round(position, 3)}")
        return position, orientation


class DepthProcessor:
    """Convert camera readings into 3D points and group them into possible objects."""
    def __init__(self,):
        # Keep roughly one point per 5 mm cube to reduce the work in later steps.
        self.voxel_size = 0.005

    def rgbd_to_pointcloud(self, rgb, depth, intrinsics, width, height):
        """Turn colour and depth pixels into coloured 3D points relative to the camera.

        Depth is in metres. intrinsics is [fx, fy, cx, cy]: the camera's focal
        lengths and image centre in pixels. width and height are the image size.
        """
        fx, fy, cx, cy = intrinsics

        # u = column number, v = row number of every pixel
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Turn each pixel and its depth into a 3D point relative to the camera.
        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        points = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
        colors = rgb.reshape(-1, 3) / 255.0

        # Keep usable depths in metres; discard missing readings and distant background.
        valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0) & (points[:, 2] < 1.5)
        points = points[valid]
        colors = colors[valid]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def transform_pointcloud(self, pointcloud:o3d.geometry.PointCloud, transform):
        """Copy the camera's 3D points and express them relative to the robot base."""
        # Express every view relative to the robot base so the clouds can be combined.
        # The 4x4 camera transform applies both rotation and translation.
        pointcloud_base = o3d.geometry.PointCloud(pointcloud)   # copy
        pointcloud_base.transform(transform)
        return pointcloud_base

    def register_pointclouds(self, pcds:list):
        """STEP 1.3: Align the views with ICP and merge them into one point cloud.

        ICP adjusts overlapping clouds to reduce the distance between matching
        points. The input clouds must already be in world coordinates.
        """
        # An empty scan should flow through the pipeline as an empty cloud.
        pcds = [pcd for pcd in pcds if len(pcd.points) > 0]
        if not pcds:
            print("No scan points collected; returning an empty point cloud")
            return o3d.geometry.PointCloud()

        # All clouds are already in world frame (we know the camera pose).
        # ICP only fixes small misalignments between views.
        scene_pointcloud = pcds[0].voxel_down_sample(self.voxel_size)
        for i in range(1, len(pcds)):
            source = pcds[i].voxel_down_sample(self.voxel_size)
            result = o3d.pipelines.registration.registration_icp(
                source, scene_pointcloud,
                max_correspondence_distance=0.02,
                init=np.eye(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
            source.transform(result.transformation)
            scene_pointcloud += source
            # downsample again so the merged cloud does not keep growing
            scene_pointcloud = scene_pointcloud.voxel_down_sample(self.voxel_size)
            print(f"ICP view {i}: fitness = {result.fitness:.3f}, rmse = {result.inlier_rmse:.4f}")

        print(f"Merged pointcloud has {len(scene_pointcloud.points)} points")
        if VISUALIZE:
            self.visualize_pointcloud([scene_pointcloud])
        return scene_pointcloud

    def remove_planes(self, scene_pcd):
        """STEP 1.4: Remove large flat surfaces, then keep points above the table."""
        pcd = scene_pcd.voxel_down_sample(self.voxel_size)
        table_height = None

        # remove big horizontal planes one by one (table top, ground floor)
        for i in range(3):
            # Fitting a plane needs at least three points, including after a removal.
            if len(pcd.points) < 3:
                break
            plane_model, plane_pcd, remaining_pcd = self.remove_plane(pcd)
            a, b, c, d = plane_model
            is_horizontal = abs(c) > 0.99                 # normal points straight up / down
            # This point-count cutoff is tuned for the 5 mm grid and needs checking on new scenes.
            is_big = len(plane_pcd.points) > 10000
            if not (is_horizontal and is_big):
                break
            plane_height = -d / c
            print(f"Removed plane {i + 1}: height z = {plane_height:.3f}, {len(plane_pcd.points)} points")
            if table_height is None or plane_height > table_height:
                table_height = plane_height
            pcd = remaining_pcd

        # everything below the table top (table edges, legs, floor) is not an object
        if table_height is not None:
            points = np.asarray(pcd.points)
            keep = np.where(points[:, 2] > table_height + 0.01)[0]
            pcd = pcd.select_by_index(keep)

        print(f"Points left after plane removal: {len(pcd.points)}")
        if VISUALIZE:
            self.visualize_pointcloud([pcd])
        return pcd

    def remove_plane(self, scene_pcd):
        """Find the largest flat surface using RANSAC, which tolerates scattered points.

        Return its plane equation, the points on it, and the remaining points.
        The input needs at least three points to define a plane.
        """
        # RANSAC: fit plane ax + by + cz + d = 0 to largest set of points
        plane_model, inliers = scene_pcd.segment_plane(distance_threshold=0.01,
                                                       ransac_n=3,
                                                       num_iterations=1000)
        plane_pcd = scene_pcd.select_by_index(inliers)
        remaining_pcd = scene_pcd.select_by_index(inliers, invert=True)
        return plane_model, plane_pcd, remaining_pcd

    def cluster_objects(self, scene_pcd):
        """STEP 1.5: Group nearby 3D points into possible objects using DBSCAN."""
        if len(scene_pcd.points) == 0:
            print("No points available to group into objects")
            return []

        # DBSCAN groups nearby points without needing to know the object count.
        # eps is a 2 cm neighbour distance; min_points rejects sparse noise.
        labels = np.array(scene_pcd.cluster_dbscan(eps=0.02, min_points=20))
        if labels.size == 0:
            return []
        num_clusters = labels.max() + 1

        clusters = []
        # Noise has label -1, so starting at 0 leaves those points out.
        for i in range(num_clusters):
            indices = np.where(labels == i)[0]
            cluster = scene_pcd.select_by_index(indices)
            center = cluster.get_axis_aligned_bounding_box().get_center()
            # ignore very small clusters (noise)
            if len(indices) < 100:
                continue
            # ignore points of the robot itself (close to robot base at origin)
            if np.linalg.norm(center[:2]) < 0.2:
                continue
            clusters.append(cluster)
            print(f"Cluster {len(clusters) - 1}: {len(indices)} points, center {np.round(center, 3)}")

        print(f"Found {len(clusters)} object clusters")
        if VISUALIZE:
            # give every cluster a random color so they are easy to see
            colored = []
            for cluster in clusters:
                c = o3d.geometry.PointCloud(cluster)
                c.paint_uniform_color(np.random.rand(3))
                colored.append(c)
            self.visualize_pointcloud(colored)
        return clusters

    def get_cluster_top_position(self, cluster):
        """Choose a camera viewpoint 30 cm above the top of a group of points."""
        bbox = cluster.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        top = bbox.get_max_bound()[2]
        # camera is a few cm away from the end effector, so shift a little to center the object
        return [center[0] - 0.04, center[1], top + 0.30]

    def project_world_to_image(self, point_world, intrinsics, T_cb):
        """
        Find the image pixel for a 3D point measured relative to the robot base.

        T_cb describes the camera's position and rotation relative to the base.
        Return (column, row), or None if the point cannot be projected.
        """
        fx, fy, cx, cy = intrinsics
        # world -> camera frame (inverse of camera pose)
        T_bc = np.linalg.inv(T_cb)
        point = np.append(point_world, 1.0)
        X, Y, Z, _ = T_bc @ point
        # A point behind the camera cannot appear in the image. Zero depth
        # would also cause division by zero in the pixel equations below.
        if not np.isfinite([X, Y, Z]).all() or Z <= 0:
            return None
        # camera frame -> pixel (pinhole model)
        u = fx * X / Z + cx
        v = fy * Y / Z + cy
        return (u, v)

    def visualize_pointcloud(self, pcds:list[o3d.geometry.PointCloud]):
        """Show the 3D points and robot-base axes. Close the window to continue."""
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)  # robot base frame
        o3d.visualization.draw_geometries(pcds + [axes])
