
from perception import Perception
import numpy as np




class Planner:
    def __init__(self, manipulator):
        self.perceptor = Perception(manipulator)

    def get_clear_task_plan(self, objs):
        '''
        STEP 2.1 - returns a sequential action plan executable by manipulator to place all the objects in the plate
        you can write the plans for specific objects such that plan steps are manipulator function calls, and robust to object location randomization
        '''
        # TODO: find the plate in objs, then for every fruit add pick + place steps
        print("get_clear_task_plan: not implemented yet")
        return []

    def pick_place_object_plan(self, obj_label, place_location_label, objs):
        '''
        returns a sequential action plan executable by manipulator to pick a object and place it at a specified location
        args
            obj_label : label of object to pick
            place_location label : label of object on which the object is to be placed
        '''
        # TODO: use perceptor.get_object_pick_pose / get_obj_place_pose to build the steps
        return []
