import numpy as np
from base import PandaArm
import time


class Manipulator(PandaArm):
    """Move the arm and gripper using the robot's own state, not object ground truth."""
    def __init__(self, p, pandaId):
        super().__init__(p, pandaId)
        self.p = self.pybullet_client
        # gripper pointing straight down (roll = 180 deg)
        self.down_orientation = [np.pi, 0, 0]

    def get_joint_positions(self,):
        """Read the seven arm joint angles, in radians."""
        joint_states = self.p.getJointStates(self.pandaId, self.REVOLUTE_JOINTS)
        return [state[0] for state in joint_states]

    def is_moving(self,):
        """Return True while any arm joint is still moving noticeably."""
        joint_states = self.p.getJointStates(self.pandaId, self.REVOLUTE_JOINTS)
        velocities = [abs(state[1]) for state in joint_states]
        # if any joint is moving faster than a small threshold, robot is still moving
        return max(velocities) > 0.01

    def move_joints(self, target_joints:list):
        """Move to seven joint angles in radians. Stop the task if the move fails."""
        target_joints = np.asarray(target_joints, dtype=float)
        if target_joints.shape != (7,) or not np.isfinite(target_joints).all():
            raise ValueError("Arm movement needs seven finite joint angles, in radians")
        if np.any(target_joints < self.JOINT_LOWER_LIMITS) or np.any(target_joints > self.JOINT_UPPER_LIMITS):
            raise ValueError("Requested joint angles exceed the Panda arm's limits; movement cancelled")

        # Check all angles first so a bad target cannot start a partial movement.
        for joint, angle in zip(self.REVOLUTE_JOINTS, target_joints):
            # maxVelocity keeps the motion slow so a grasped object does not fly away
            self.p.setJointMotorControl2(self.pandaId, joint, self.p.POSITION_CONTROL,
                                         targetPosition=angle, force=200, maxVelocity=0.8)
        # Wait for both the target angles and a steady arm. A monotonic clock
        # keeps this time limit independent of changes to the computer's clock.
        start_time = time.monotonic()
        while time.monotonic() - start_time < 10:
            current = np.array(self.get_joint_positions())
            error = np.max(np.abs(current - np.array(target_joints)))
            if error < 0.01 and not self.is_moving():
                return
            time.sleep(0.01)

        # Cancel the old target by holding the current angles. Raising an error
        # prevents later actions, such as closing the gripper at the wrong place.
        for joint, angle in zip(self.REVOLUTE_JOINTS, self.get_joint_positions()):
            self.p.setJointMotorControl2(self.pandaId, joint, self.p.POSITION_CONTROL,
                                         targetPosition=angle, force=200, maxVelocity=0.8)
        raise TimeoutError("The arm did not reach its target within 10 seconds. Holding position and stopping the task.")

    def move_home(self,):
        """Return to the starter code's home joint angles."""
        self.move_joints(self.PANDA_HOME_POSITION)

    def move_in_task_space(self, target_position, target_orientation=None):
        """Move the hand to [x, y, z] in metres, relative to the robot base.

        Orientation is [roll, pitch, yaw] in radians; the default points down.
        Joint limits are checked, but this is not an obstacle-aware path planner.
        """
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
        """Open or close both fingers and allow one second for movement.

        True means the command was sent; it does not confirm a successful grasp.
        """
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
        """Approach the sensed object from above, grip it, and lift it.

        Position is in metres; orientation is [roll, pitch, yaw] in radians.
        """
        x, y, z = obj_location
        # crane grasp: open -> go above -> go down -> close -> lift up
        self.command_gripper(open=True)
        self.move_in_task_space([x, y, z + 0.20], obj_orientation)
        self.move_in_task_space([x, y, z + 0.05], obj_orientation)
        self.move_in_task_space([x, y, z], obj_orientation)
        self.command_gripper(open=False)
        self.move_in_task_space([x, y, z + 0.25], obj_orientation)

    def place_object_at(self, target_location:list, target_orientation:list):
        """Move above the plate, lower the held object, release it, then lift away."""
        x, y, z = target_location
        # go above the target, go down a little, open fingers, move up again
        self.move_in_task_space([x, y, z + 0.15], target_orientation)
        self.move_in_task_space([x, y, z], target_orientation)
        self.command_gripper(open=True)
        self.move_in_task_space([x, y, z + 0.15], target_orientation)

    def execute_task_plan(self, plan:list):
        """STEP 2.2: Run the planner's actions in order; errors stop the task."""
        if not plan:
            print("No actions to perform")
            return
        # every step is (function_name, list_of_arguments)
        for step_number, (action, args) in enumerate(plan):
            print(f"Step {step_number + 1}/{len(plan)}: {action.replace('_', ' ')}")
            function = getattr(self, action)
            function(*args)
        print("All planned movements finished; placement success has not been checked")
