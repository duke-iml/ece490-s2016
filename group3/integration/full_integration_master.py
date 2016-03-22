import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

import rospy
import random
import time
import thread
from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from OpenGL.GL import *
from planning.MyGLViewer import MyGLViewer
from util.constants import *
from Motion import motion
from perception import perception
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2

baxter_rest_config = [0.0]*54

class Milestone1Master:
    def __init__(self, world):
        self.world = world
        self.robotModel = world.robot(0)
        self.state = INITIAL_STATE
        self.config = self.robotModel.getConfig()
        self.left_arm_links = [self.robotModel.link(i) for i in LEFT_ARM_LINK_NAMES]
        self.right_arm_links = [self.robotModel.link(i) for i in RIGHT_ARM_LINK_NAMES]
        id_to_index = dict([(self.robotModel.link(i).getID(),i) for i in range(self.robotModel.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

        self.Tcamera = se3.identity()
        self.Tvacuum = se3.identity()
        self.object_com = [0, 0, 0]
        self.points = []

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
            goal = ik.objective(self.robotModel.link('right_wrist'),local=VACUUM_POINT_XFORM[1],world=right_target)
            if ik.solve(goal,tol=0.0001):
                return True
        print "right_arm_ik failed for ", right_target
        return False

    def start(self):
        motion.setup(mode='physical',klampt_model=os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"),libpath=LIBPATH)
        motion.robot.startup()
        rospy.init_node("listener", anonymous=True)
        self.loop()

    def drawStuff(self):
        gldraw.xform_widget(self.Tvacuum,0.1,0.01)
        glPointSize(5.0)
        glBegin(GL_POINTS)
        for point in self.points[::100]:
            transformed = se3.apply(self.Tcamera, point)
            glVertex3f(transformed[0], transformed[1], transformed[2])
        # glVertex3f(self.object_com[0], self.object_com[1], self.object_com[2])
        glEnd()

    def loop(self):
        try:
            while True:
                print self.state
                self.load_real_robot_state()
                self.Tcamera = se3.mul(self.robotModel.link('right_lower_elbow').getTransform(), RIGHT_F200_CAMERA_CALIBRATED_XFORM)
                self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)

                if self.state == 'CUSTOM_CODE':
                    print se3.apply(self.Tvacuum, [0, 0, 0])

                elif self.state == 'START':
                    motion.robot.left_mq.setLinear(3, [0, 0, 0, 0, 0, 0, 0])
                    self.state = 'MOVING_LEFT_TO_0'
                elif self.state == 'MOVING_LEFT_TO_0':
                    if not motion.robot.left_mq.moving():
                        motion.robot.right_mq.appendLinear(3, Q_INTERMEDIATE_1)
                        motion.robot.right_mq.appendLinear(3, [0, 0, 0, 0, 0, 0, 0])
                        self.state = 'MOVING_RIGHT_TO_0'
                elif self.state == 'MOVING_RIGHT_TO_0':
                    if not motion.robot.right_mq.moving():
                        motion.robot.right_mq.appendLinear(3, Q_INTERMEDIATE_1)
                        motion.robot.right_mq.appendLinear(3, Q_SCAN_BIN)
                        self.state = 'MOVING_TO_SCAN_BIN'
                elif self.state == 'MOVING_TO_SCAN_BIN':
                    if not motion.robot.right_mq.moving():
                        self.wait_start_time = time.time()
                        self.state = 'WAITING_TO_SCAN_BIN'
                elif self.state == 'WAITING_TO_SCAN_BIN':
                    if time.time() - self.wait_start_time > SCAN_WAIT_TIME:
                        motion.robot.right_mq.appendLinear(3, Q_INTERMEDIATE_2)
                        self.state = 'FAKE_SCANNING_BIN'
                elif self.state == 'SCANNING_BIN':
                    cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
                    if perception.isCloudValid(cloud):
                        # These are in the camera frame
                        self.points, self.object_com = perception.getObjectCOM(cloud)
                        self.state = 'MOVING_TO_GRASP_OBJECT'
                    else:
                        print "Got an invalid cloud, trying again"
                elif self.state == 'FAKE_SCANNING_BIN':
                    self.object_com = [1.174, -0.097, 1.151]
                    self.points = [self.object_com]
                    self.state = 'MOVING_TO_GRASP_OBJECT'
                    if self.right_arm_ik(self.object_com):
                        destination = self.robotModel.getConfig()
                        motion.robot.right_mq.appendLinear(3, [destination[v] for v in self.right_arm_indices])
                        self.state = 'MOVING_TO_GRASP_OBJECT'
                    else:
                        print "Couldn't move there"
                elif self.state == 'MOVING_TO_GRASP_OBJECT':
                    if not motion.robot.right_mq.moving():
                        self.state = 'GRASPING_OBJECT'
                        print "Turn on the vacuum now"
                elif self.state == 'GRASPING_OBJECT':
                    time.sleep(5)
                    self.state = 'MOVING_BACK'
                elif self.state == 'MOVING_BACK':
                    motion.robot.right_mq.setLinear(2, Q_INTERMEDIATE_2)
                elif self.state == 'DONE':
                    print "desired COM: ", self.object_com
                    print "actual vacuum point: ", se3.apply(self.Tvacuum, [0, 0, 0])
                else:
                    print "Unknown state"


                time.sleep(1)
        except KeyboardInterrupt:
            motion.robot.stopMotion()
            sys.exit(0)

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

def visualizerThreadFunction():
    visualizer.run()

if __name__ == '__main__':
    world = setupWorld()
    master = Milestone1Master(world)
    visualizer = MyGLViewer(world, master)

    thread.start_new_thread(visualizerThreadFunction, ())
    master.start()
