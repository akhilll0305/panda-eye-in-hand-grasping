from base import RGBDSensor
from control import Manipulator

from ultralytics import YOLO
import open3d as o3d
import numpy as np
import cv2
import os

# set this to True to see the pointcloud after every step (useful for demo)
VISUALIZE = True
# detection images from identify_objects are saved in this folder
OUTPUT_FOLDER = "./outputs"

# Inside each class the functions are written in the order they are used in main.py.
# "STEP x.y" in a docstring is the step number from main.py.


class ObjectDetector:
    def __init__(self,):
        self.detector = YOLO("./weights/best.pt")

    def detect_objects(self, rgbImage):
        '''
        returns obj_labels and 2D bounding box the objects in the rgb image
        args
            rgbImage : RGB image containing object to be identified

        please note that due to openCV default BGR convention, you will need to
        first convert the image to BGR before feeding to the detector
        '''
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
    def __init__(self, manipulator:Manipulator):
        self.sensor = RGBDSensor()
        self.obj_detector = ObjectDetector()
        self.manipulator = manipulator
        self.depth_processor = DepthProcessor()

    def get_observation(self,):
        '''
        captures rgb, depth, camera pose and intrinsics from the hand camera
        '''
        p = self.manipulator.pybullet_client
        return self.sensor.get_observation(p, self.manipulator.pandaId)

    def generate_exploration_poses(self,):
        '''
        STEP 1.1 - returns list of 7D joint poses to explore the scene
        '''
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
        '''
        STEP 1.2 - explore the scene using generated exploration poses and return collected pointclouds
        args
            exploration_poses : list with set of proposed 7D joint positions fullfilling environment/scene exploration
        returns
            pointclouds : list of collected pointclouds from the exploration
        '''
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
        '''
        STEP 1.6 - assigns labels to all object clusters in the pointcloud_clusters
        args
            pointcloud_clusters : clustered pointclouds
        returns
            objs : dictionary containing labels and spatial information of all the objects in the scene
                    {obj_label: obj_pointcloud}
        '''
        # TODO: for every cluster
        #   1. move camera above the cluster and take a picture
        #   2. run YOLO (ObjectDetector.detect_objects) on it
        #   3. project cluster center into the image, match it with a YOLO box
        #   4. store {label: cluster}
        print("identify_objects: not implemented yet")
        return {}

    def get_object_pick_pose(self, obj_label:str, objs:dict):
        '''
        used by planner (STEP 2.1) - returns 6D pose of the queried object: obj_label
        args
            obj_label : query object label
            objs : dictionary containing labels and spatial information of all the objects in the scene
        '''
        # TODO: grasp at cluster centroid (x, y), a little below the top of the object
        pass

    def get_obj_place_pose(self, obj_label:str, target_obj_label:str, objs:dict):
        '''
        used by planner (STEP 2.1) - returns 6D target pose for object to be placed at target object location
        args
            obj_label : to be placed object label
            target_obj_label : label of the object on which object is to be placed
            objs : dictionary containing labels and spatial information of all the objects in the scene
        '''
        # TODO: release a little above the center of the plate
        pass


class DepthProcessor:
    def __init__(self,):
        self.voxel_size = 0.005   # 5 mm grid used for downsampling

    def rgbd_to_pointcloud(self, rgb, depth, intrinsics, width, height):
        '''
        used in STEP 1.2 - returns a colored pointcloud in the camera frame
        args
            rgb : RGB image
            depth : depth image
            intrinsics : list - [fx, fy, cx, cy] : intrinsic parameters from k matrix
        '''
        fx, fy, cx, cy = intrinsics

        # u = column number, v = row number of every pixel
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # pinhole camera model
        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        points = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
        colors = rgb.reshape(-1, 3) / 255.0

        # remove far away points (background / max range of sensor)
        valid = points[:, 2] < 1.5
        points = points[valid]
        colors = colors[valid]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def transform_pointcloud(self, pointcloud:o3d.geometry.PointCloud, transform):
        '''
        used in STEP 1.2 - returns a pointcloud transformed in the target transform frame
        args
            pointcloud :
            transform :
        returns
            pointcloud_base : pointcloud transformed in the robot base frame
        '''
        # world_point = T_cb * camera_point   (4x4 homogeneous transform)
        pointcloud_base = o3d.geometry.PointCloud(pointcloud)   # copy
        pointcloud_base.transform(transform)
        return pointcloud_base

    def register_pointclouds(self, pcds:list):
        '''
        STEP 1.3 - returns a poincloud aligned in a single frame
        args
            pcds : list of pointclouds
        returns
            scene_pointcloud : combined and aligned pointcloud in a single frame from pcds
        '''
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
        '''
        STEP 1.4 - this function iteratively removes multiple planes from the scene_pcd
        '''
        pcd = scene_pcd.voxel_down_sample(self.voxel_size)
        table_height = None

        # remove big horizontal planes one by one (table top, ground floor)
        for i in range(3):
            plane_model, plane_pcd, remaining_pcd = self.remove_plane(pcd)
            a, b, c, d = plane_model
            is_horizontal = abs(c) > 0.99                 # normal points straight up / down
            is_big = len(plane_pcd.points) > 10000        # table / floor are huge, objects are small
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
        '''
        used in STEP 1.4 - this function removes a plane that maximally fits a plane equation (criterion)
        args
            scene_pcd : pointcloud of a scene
        '''
        # RANSAC: fit plane ax + by + cz + d = 0 to largest set of points
        plane_model, inliers = scene_pcd.segment_plane(distance_threshold=0.01,
                                                       ransac_n=3,
                                                       num_iterations=1000)
        plane_pcd = scene_pcd.select_by_index(inliers)
        remaining_pcd = scene_pcd.select_by_index(inliers, invert=True)
        return plane_model, plane_pcd, remaining_pcd

    def cluster_objects(self, scene_pcd):
        '''
        STEP 1.5 - this function clusters the pointcloud
        args
            scene_pcd : merged pointcloud of the scene
        '''
        # DBSCAN: points closer than eps are neighbours, need min_points to form a cluster
        labels = np.array(scene_pcd.cluster_dbscan(eps=0.02, min_points=20))
        num_clusters = labels.max() + 1

        clusters = []
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
        '''
        used in STEP 1.6 - position above the cluster where the camera takes a picture
        args
            cluster : a cluster from the clustered pointcloud which manipulator aims to understand
        '''
        # TODO
        pass

    def project_world_to_image(self, point_world, intrinsics, T_cb):
        """
        used in STEP 1.6 - world point -> pixel (reverse of rgbd_to_pointcloud)
        args:
            point_world : (x,y,z) in robot base (world) frame
            intrinsics : list [fx, fy, cx, cy] - camera intrinsic parameters
            T_cb: (4x4) transform of camera frame wrt robot base frame
        returns:
            (u, v): pixel coordinates
        """
        # TODO: inverse of rgbd_to_pointcloud (world -> camera frame -> pixel)
        pass

    def visualize_pointcloud(self, pcds:list[o3d.geometry.PointCloud]):
        '''
        this function will visualize the pointcloud(s) present in the list `pcds`
        args
            pcds: list of pointclouds to be visualized (could be single/multiple pointclouds)
        '''
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)  # robot base frame
        o3d.visualization.draw_geometries(pcds + [axes])
