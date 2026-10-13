
from perception import Perception
import numpy as np




class Planner:
    def __init__(self, manipulator):
        self.perceptor = Perception(manipulator)

    def get_clear_task_plan(self, objs):
        '''
        returns a sequential action plan executable by manipulator to place all the objects in the plate
        you can write the plans for specific objects such that plan steps are manipulator function calls, and robust to object location randomization
        '''
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

        # small shift for every fruit so they do not fall on top of each other in the plate
        offsets = [[0.04, 0], [-0.04, 0], [0, 0.04], [0, -0.04], [0, 0]]

        plan = [("move_home", [])]
        for i, fruit in enumerate(fruits):
            offset = offsets[i % len(offsets)]
            plan += self.pick_place_object_plan(fruit, plate_label, objs, offset)
        return plan

    def pick_place_object_plan(self, obj_label, place_location_label, objs, offset=[0, 0]):
        '''
        returns a sequential action plan executable by manipulator to pick a object and place it at a specified location
        args
            obj_label : label of object to pick
            place_location label : label of object on which the object is to be placed
        '''
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
