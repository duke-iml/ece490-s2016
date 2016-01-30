from klampt import se3, so3

from apc.motion import PhysicalLowLevelController
from hardware_layer.Motion import motion
from klampt_models import reflex_col as gripper
import apc.baxter_scoop as baxter

class KnowledgeBase:
    max_drop_height = 0.3

    order_bin_tolerance = 0.1
    vantage_point_tolerance = 0.01

    left_camera_offset_xform = baxter.left_camera_center_xform 
    right_camera_offset_xform = baxter.right_camera_center_xform 

    def __init__(self):
        # maps from bin name to lists of object names
        self.starting_bin_contents = {}
        self.bin_contents = {}

        # map from bin names to (min, max) in SHELF LOCAL coordinates
        # NOTE: these are in shelf local coordinates since the shelf can move
        self.bin_bounds = {}        
        # map from bin name to ordered list of vantage point name
        # list is ordered by vantage point preference
        self.bin_vantage_points = {}
        # map from vantage point names to (R, t) in SHELF LOCAL coordinates
        # NOTE: these are in shelf local coordinates since the shelf can move
        self.vantage_point_xforms = {}
        # map from bin names to (R, t) in SHELF LOCAL coordinates
        # NOTE: these are in shelf local coordinates since the shelf can move
        self.withdraw_point_xforms = {}

        # transforms of shelf and order bin in WORLD coordinates
        self.shelf_xform = None
        self.order_bin_xform = None

        # transforms of the objects in WORLD coordinates as available
        # map from object_name to (R, t)
        self.object_xforms = {}
        # point clouds of objects detected in bins
        # points are given in WORLD coordinates
        # map from object name to list of 4-tuples (x, y, z, (R, G, B))
        self.object_clouds = {}

        #minimum and maximum object dimensions
        self.object_minimum_dims = {}
        self.object_maximum_dims = {}
        self.object_median_dims = {}

        # confusion matrix for last perception operation
        # map from object names in the target bin to probabilities
        self.last_confusion_matrix = None

        # the limb that performed the last motion
        # 'right', 'left', or None
        self.active_limb = None
        # the name of the object grasped or None
        self.grasped_object = None
        self.active_grasp = None

        # the bin and object names that the master controller is attempting
        self.target_bin = None
        self.target_object = None

        # the current state of the robot
        self.robot_state = None

class RobotState:
    def __init__(self, controller):
        self.sensed_config = controller.getSensedConfig()
        self.sensed_velocity = [ 0 ] * len(self.sensed_config) # controller.getSensedVelocity()

        self.commanded_config = controller.getCommandedConfig()
        self.commanded_velocity = [ 0 ] * len(self.commanded_config) # controller.getCommandedVelocity()
        
        if isinstance(controller, PhysicalLowLevelController):
            for (gripper, limb) in [ (motion.robot.left_gripper, 'left'), (motion.robot.right_gripper, 'right') ]:
                if gripper.position() != False:
                    baxter.set_q_gripper_command(self.sensed_config, limb, gripper.position())
                if gripper.target() != False:
                    baxter.set_q_gripper_command(self.commanded_config, limb, gripper.target())

    def __str__(self):
        return '{} -> {}\n{} -> {}'.format(self.commanded_config, self.sensed_config, self.commanded_velocity, self.commanded_config)
