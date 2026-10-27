import numpy as np
from base import PandaArm
import time


class Manipulator(PandaArm):
    def __init__(self, p, pandaId):
        super().__init__(p, pandaId)
        self.p = self.pybullet_client
        # gripper pointing straight down (roll = 180 deg)
        self.down_orientation = [np.pi, 0, 0]

    def get_joint_positions(self,):
        '''
        returns current angles of the 7 arm joints (robot's own state, allowed)
        '''
        joint_states = self.p.getJointStates(self.pandaId, self.REVOLUTE_JOINTS)
        return [state[0] for state in joint_states]

    def is_moving(self,):
        '''
        checks if the manipulator is steady or still in motion
        returns
            moving : bool - ?(manipulator_moving)
        '''
        joint_states = self.p.getJointStates(self.pandaId, self.REVOLUTE_JOINTS)
        velocities = [abs(state[1]) for state in joint_states]
        # if any joint is moving faster than a small threshold, robot is still moving
        return max(velocities) > 0.01

    def move_joints(self, target_joints:list):
        '''
        moves manipulator joints to target joint poses
        args
            target_joints : 7D list of target joint angles
        '''
        for joint, angle in zip(self.REVOLUTE_JOINTS, target_joints):
            # maxVelocity keeps the motion slow so a grasped object does not fly away
            self.p.setJointMotorControl2(self.pandaId, joint, self.p.POSITION_CONTROL,
                                         targetPosition=angle, force=200, maxVelocity=0.8)
        # wait till robot reaches the target (or timeout of 10 seconds)
        start_time = time.time()
        while time.time() - start_time < 10:
            current = np.array(self.get_joint_positions())
            error = np.max(np.abs(current - np.array(target_joints)))
            if error < 0.01 and not self.is_moving():
                break
            time.sleep(0.01)

    def move_home(self,):
        '''
        moves robot to home position
        '''
        self.move_joints(self.PANDA_HOME_POSITION)

    def move_in_task_space(self, target_position, target_orientation=None):
        '''
        moves robot end effector to target pose
        args
            target_position : 3D spatial location
            target_orientation : 3D orientation
        '''
        if target_orientation is None:
            target_orientation = self.down_orientation
        target_quaternion = self.p.getQuaternionFromEuler(target_orientation)

        # inverse kinematics: find joint angles that put end effector at target pose
        joint_ranges = [u - l for u, l in zip(self.JOINT_UPPER_LIMITS, self.JOINT_LOWER_LIMITS)]
        joint_angles = self.p.calculateInverseKinematics(self.pandaId, self.PANDA_END_EFFECTOR_INDEX,
                                                         target_position, target_quaternion,
                                                         lowerLimits=self.JOINT_LOWER_LIMITS,
                                                         upperLimits=self.JOINT_UPPER_LIMITS,
                                                         jointRanges=joint_ranges,
                                                         restPoses=self.PANDA_HOME_POSITION,
                                                         maxNumIterations=200,
                                                         residualThreshold=1e-4)
        # IK returns values for all movable joints (7 arm + 2 fingers), we need only first 7
        self.move_joints(list(joint_angles[:7]))

    def command_gripper(self, open:bool):
        '''
        args
            open: do we want the robot to open its fingers, otherwise close them
        returns
            bool (success/failure)

        This function will simply close or open robots gripper, so to grasp or release the object
        '''
        if open:
            target = self.PANDA_FINGER_OPEN
        else:
            target = self.PANDA_FINGER_CLOSED
        for joint in self.PANDA_PRISMATIC_JOINTS:
            self.p.setJointMotorControl2(self.pandaId, joint, self.p.POSITION_CONTROL,
                                         targetPosition=target, force=300)
        time.sleep(1.0)  # give fingers time to open / close
        return True

    def pick_object_from(self, obj_location:list, obj_orientation:list):
        '''
        perform pick operation for object placed at obj_location, obj_orientation
        args
            obj_location : object location
            obj_orientation : object orienation in euler notation
        '''
        x, y, z = obj_location
        # crane grasp: open -> go above -> go down -> close -> lift up
        self.command_gripper(open=True)
        self.move_in_task_space([x, y, z + 0.20], obj_orientation)
        self.move_in_task_space([x, y, z + 0.05], obj_orientation)
        self.move_in_task_space([x, y, z], obj_orientation)
        self.command_gripper(open=False)
        self.move_in_task_space([x, y, z + 0.25], obj_orientation)

    def place_object_at(self, target_location:list, target_orientation:list):
        '''
        places the grasped object at target location in target orientation
        '''
        x, y, z = target_location
        # go above the target, go down a little, open fingers, move up again
        self.move_in_task_space([x, y, z + 0.15], target_orientation)
        self.move_in_task_space([x, y, z], target_orientation)
        self.command_gripper(open=True)
        self.move_in_task_space([x, y, z + 0.15], target_orientation)

    def execute_task_plan(self, plan:list):
        '''
        STEP 2.2 - executes a task plan generated by the planner
        args
            plan : list of manipulator executable action steps
        '''
        # every step is (function_name, list_of_arguments)
        for step_number, (action, args) in enumerate(plan):
            print(f"Step {step_number + 1}/{len(plan)}: {action}")
            function = getattr(self, action)
            function(*args)
        print("Task plan finished")
