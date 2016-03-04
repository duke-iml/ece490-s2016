import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

import rospy
import random
from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from planning.MyGLViewer import MyGLViewer
from util.constants import *
from Motion import motion
from perception.pc import PCProcessor

baxter_rest_config = [0.0]*54

class Milestone1Master:
    def __init__(self, world):
        self.world = world
        self.robotModel = world.robot(0)
        self.config = self.robotModel.getConfig()
        self.right_camera_link = self.robotModel.link(RIGHT_CAMERA_LINK_NAME)
        self.right_gripper_link = self.robotModel.link(RIGHT_GRIPPER_LINK_NAME)
        self.left_arm_links = [self.robotModel.link(i) for i in LEFT_ARM_LINK_NAMES]
        self.right_arm_links = [self.robotModel.link(i) for i in RIGHT_ARM_LINK_NAMES]
        id_to_index = dict([(self.robotModel.link(i).getID(),i) for i in range(self.robotModel.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

    def load_real_robot_state(self):
        """Makes the robot model match the real robot"""
        self.robotModel.setConfig(motion.robot.getKlamptSensedPosition())

    def right_arm_ik(self, right_target):
        """Solves IK to move the right arm to the specified
            right_target ([x, y, z] in world space)
        """
        qmin,qmax = self.robotModel.getJointLimits()
        for i in range(100):
            q = baxter_rest_config[:]
            for j in self.right_arm_indices:
                q[j] = random.uniform(qmin[j],qmax[j])
            #goal = ik.objective(self.right_gripper_link,local=[vectorops.sub(right_gripper_center_xform[1],[0,0,0.1]),right_gripper_center_xform[1]],world=[vectorops.add(target,[0,0,0.1]),target])
            goal = ik.objective(self.right_gripper_link,local=RIGHT_GRIPPER_CENTER_XFORM[1],world=right_target)
            if ik.solve(goal,tol=0.1):
                return True
        print "right_arm_ik failed for ", right_target
        return False

    def start(self):
        motion.setup(mode='physical',klampt_model=os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"),libpath="../../common/")
        motion.robot.startup()

        # print se3.apply(self.robotModel.link('right_wrist').getTransform(), [1, 0, 0])
        # self.right_arm_ik([.5, -.25, 1])
        # destination = self.robotModel.getConfig()
        # motion.robot.right_mq.setLinear(3, [destination[v] for v in self.right_arm_indices])

        #pc_processor = PCProcessor()
        #rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        #rospy.spin()

        print "Moving Left limb to 0"
        motion.robot.left_mq.setLinear(3, [0.0] * 7)

        self.load_real_robot_state()

        # print "Moving right limb to 0"
        # limb_right.move_to_joint_positions(Q_RIGHT_ZEROS)

        # print "Scanning bin for point cloud"
        # limb_right.move_to_joint_positions(Q_SCAN_BIN)
        # cloud = "hi"
        # cloud = pc_processor.subtractShelf(cloud)
        # centroid = pc_processor.getCentroid(cloud)

        # print "Moving to centroid of cloud"
        # Calculate IK for cloud centroid
        # Move to the IK solution

        # print "Moving spatula to bin"
        # limb_left.move_to_joint_positions(Q_SPATULA_AT_BIN)

        # print "Scanning spatula"
        # limb_right.move_to_joint_positions(Q_SCAN_SPATULA)

def setupWorld():
    world = WorldModel()
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"kiva_pod/model.obj"))
    print "Loading plane model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
    T = world.rigidObject(0).getTransform()
    world.rigidObject(0).setTransform(*se3.mul(Trel,T))

    return world

if __name__ == '__main__':
    world = setupWorld()

    # Start master controller
    master = Milestone1Master(world)
    master.start()

    # Start visualizer for world model
    visualizer = MyGLViewer(world)
    visualizer.run()
