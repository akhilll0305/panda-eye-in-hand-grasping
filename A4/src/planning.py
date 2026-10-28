
from perception import Perception

class Planner:
    """Build a list of movements from the objects found by the camera."""
    def __init__(self, manipulator):
        self.perceptor = Perception(manipulator)

    def get_clear_task_plan(self, objs):
        """STEP 2.1: Plan a pick and place for each fruit, using sensed positions.

        Return an empty plan if the plate is missing. The scene has at most one
        object of each type; the wooden block stays in place as an obstacle.
        """
        # find the plate among the detected objects
        plate_label = None
        for label in objs:
            if label.startswith("plate"):
                plate_label = label
        if plate_label is None:
            print("Plate not found, cannot make a plan")
            return []

        # every fruit has to go to the plate (plate and wooden block are not moved)
        fruits = [label for label in objs if not label.startswith("plate") and not label.startswith("wood_block")]
        print("Objects to put in plate:", fruits)

        # Spread the release positions by 4 cm to reduce stacking on the plate.
        offsets = [[0.04, 0], [-0.04, 0], [0, 0.04], [0, -0.04], [0, 0]]

        plan = [("move_home", [])]
        for i, fruit in enumerate(fruits):
            offset = offsets[i % len(offsets)]
            plan += self.pick_place_object_plan(fruit, plate_label, objs, offset)
        return plan

    def pick_place_object_plan(self, obj_label, place_location_label, objs, offset=(0, 0)):
        """Plan one fruit's pick, return home, place, and return home again.

        offset shifts the release position in x and y, measured in metres.
        Returning home reduces collision risk, but does not guarantee a clear path.
        """
        pick_position, pick_orientation = self.perceptor.get_object_pick_pose(obj_label, objs)
        place_position, place_orientation = self.perceptor.get_obj_place_pose(obj_label, place_location_label, objs)
        place_position = [place_position[0] + offset[0], place_position[1] + offset[1], place_position[2]]

        # each step = (name of Manipulator function, arguments of that function)
        plan = [
            ("pick_object_from", [pick_position, pick_orientation]),
            ("move_home", []),
            ("place_object_at", [place_position, place_orientation]),
            ("move_home", []),
        ]
        return plan
