import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

import rospy
import random
import time
import serial
import thread
from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from OpenGL.GL import *
from planning.MyGLViewer import MyGLViewer
from util.constants import *
from Motion import motion
from perception import perception
from planning import planning
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2

baxter_rest_config = [0.0]*54

class FullIntegrationMaster:
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
        self.points1 = []
        self.points2 = []
        self.cameraCalibration = RIGHT_F200_CAMERA_CALIBRATED_XFORM

        self.vacuumPc = Geometry3D()
        self.vacuumPc.loadFile(VACUUM_PCD_FILE)

        # Set up serial
        if REAL_VACUUM:
            self.serial = serial.Serial()
            self.serial.port = ARDUINO_SERIAL_PORT
            self.serial.baudrate = 9600
            self.serial.open()
            if self.serial.isOpen():
                self.serial.write("hello")
                response = self.serial.read(self.serial.inWaiting())

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
        glDisable(GL_LIGHTING)
        gldraw.xform_widget(self.Tvacuum,0.1,0.01)
        gldraw.xform_widget(self.Tcamera,0.1,0.01)

        glPointSize(5.0)
        glColor3f(0.0,0.0,1.0)
        glBegin(GL_POINTS)
        for point in self.points1[::25]:
            glVertex3f(point[0], point[1], point[2])
        glEnd()

        glColor3f(1.0,0.0,0.0)
        glBegin(GL_POINTS)
        for point in self.points2[::25]:
            glVertex3f(point[0], point[1], point[2])
        glEnd()

        glPointSize(20.0)
        glColor3f(0.0,0.8,0.0)
        glBegin(GL_POINTS)
        glVertex3f(self.object_com[0], self.object_com[1], self.object_com[2])
        glEnd()

        glPointSize(5.0)
        glColor3f(1.0,0.0,0.0)
        glBegin(GL_POINTS)
        for i in range(self.vacuumPc.getPointCloud().numPoints()):
            point = self.vacuumPc.getPointCloud().getPoint(i)
            glVertex3f(point[0], point[1], point[2])
        glEnd()

    def turnOnVacuum(self):
        if REAL_VACUUM:
            self.serial.write('H')
        else:
            print "Fake vacuum is on"

    def turnOffVacuum(self):
        if REAL_VACUUM:
            self.serial.write('L')
        else:
            print "Fake vacuum is off"

    def calibrateCamera(self):
        print self.cameraCalibration

        calibrateR = self.cameraCalibration[0];
        calibrateT = self.cameraCalibration[1];

        try:
            input_var = raw_input("Enter joint and angle to change to separated by a comma: ").split(',');
            #translational transformation
            if(input_var[0] == "x" ):
                calibrateT[0] = calibrateT[0] + float(input_var[1])
            elif(input_var[0] == "y" ):
                calibrateT[1] = calibrateT[1] + float(input_var[1])
            elif(input_var[0] == "z" ):
                calibrateT[2] = calibrateT[2] + float(input_var[1])
            #rotational transformations
            elif(input_var[0] == "xr" ):
                calibrateR = so3.mul(calibrateR, so3.rotation([1, 0, 0], float(input_var[1])))
            elif(input_var[0] == "yr" ):
                calibrateR = so3.mul(calibrateR, so3.rotation([0, 1, 0], float(input_var[1])))
            elif(input_var[0] == "zr" ):
                calibrateR = so3.mul(calibrateR, so3.rotation([0, 0, 1], float(input_var[1])))

            time.sleep(0.1);

            self.cameraCalibration = (calibrateR, calibrateT)

        except: 
            print "input error\n"

            print "printing camera calibration"
            print self.cameraCalibration

        try: 
            move = raw_input("Do you want to move? (y or n)");
            if move == "y":
                self.state = 'MOVING_TO_GRASP_OBJECT'
        except:
            print "input error\n"

   
    def motionPlanArm(self, start, goal, limb):
        planner = planning.LimbPlanner(self.world)
        plannedPath = planner.plan_limb(limb, start, goal)
        while not plannedPath:
            print "trying planning again"
        for limbMilestone in plannedPath:
            if(len(limbMilestone) > 7):
                limbMilestone = [limbMilestone[v] for v in self.right_arm_indices]
            print "limbMilestone", limbMilestone
            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(limbMilestone))
        sys.stdout.flush()

    def loop(self):
        try:
            while True:
                print self.state

                self.load_real_robot_state()
                self.Tcamera = se3.mul(self.robotModel.link('right_lower_forearm').getTransform(), RIGHT_F200_CAMERA_CALIBRATED_XFORM)
                self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)

                self.vacuumPc = Geometry3D()
                self.vacuumPc.loadFile(VACUUM_PCD_FILE)
                temp_xform = self.robotModel.link('right_wrist').getTransform()
                self.vacuumPc.transform(self.Tvacuum[0], self.Tvacuum[1])

                if self.state == 'CUSTOM_CODE':
                    pass

                elif self.state == 'START':
                    #motion.robot.right_mq.appendLinear(MOVE_TIME, Q_SCAN_BIN)
                    self.state = 'MOVING_TO_SCAN_BIN'
                elif self.state == 'MOVING_TO_SCAN_BIN':
                    if not motion.robot.right_mq.moving():
                        self.wait_start_time = time.time()
                        self.state = 'WAITING_TO_SCAN_BIN'
                elif self.state == 'WAITING_TO_SCAN_BIN':

                    if time.time() - self.wait_start_time > SCAN_WAIT_TIME:
                        if REAL_PERCEPTION and REAL_PLANNING:
                            self.state = 'SCANNING_BIN'
                        elif not REAL_PLANNING:
                            self.state = 'FAKE_PATH_PLANNING'
                        else:
                            self.state = 'FAKE_SCANNING_BIN'
                elif self.state == 'SCANNING_BIN':
                    print "Waiting for message from camera"    
                    cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
                   
                    if perception.isCloudValid(cloud):
                        np_cloud = perception.convertPc2ToNp(cloud)

                        self.points1 = []
                        for point in np_cloud:
                            transformed = se3.apply(self.Tcamera, point)
                            self.points1.append(transformed)

                        np_cloud = perception.subtractShelf(np_cloud)
                    
                        self.points2 = []
                        for point in np_cloud:
                            transformed = se3.apply(self.Tcamera, point)
                            self.points2.append(transformed)

                        # plane = perception.segmentationtest(np_cloud) # TODO chenyu is fixing
                        self.object_com = se3.apply(self.Tcamera, perception.com(np_cloud))

                        if CALIBRATE:
                            self.calibrateCamera()
                        else:
                            start = self.robotModel.getConfig()
                            if self.right_arm_ik(self.object_com):
                                destination = self.robotModel.getConfig()
                                self.motionPlanArm(start, destination, 'right')
                                self.state = 'DONE'
                    else:
                        print "Got an invalid cloud, trying again"
                elif self.state == 'FAKE_PATH_PLANNING':
                    test_planner = planning.LimbPlanner(self.world, self.vacuumPc)
                    print '==========='
                    print test_planner.check_collision_free('right')
                    print '==========='
                    sys.stdout.flush()

                    # start = self.robotModel.getConfig()
                    # destination = [1.2923788123168947, -0.5817622131408692,  0.6699661083435059,  0.9123350725524902, 1.1884516140563965, 1.1524030655822755, -1.8806604437988284]
                    # destination = [0.0, 0.0,0.0,0.0,0.0,0.0,0.0]
                    # #destination = [1.0166457660095216, -0.4993107458862305, -0.23508255547485354, 0.8578787546447755, 0.2534903249084473, -0.33172334500122075, -0.20823789171752932]
                    # self.motionPlanArm(start, destination, 'right')
                    # self.state = 'DONE'

                elif self.state == 'FAKE_SCANNING_BIN':
                    self.object_com = [1.174, -0.097, 1.251]
                    self.points = [self.object_com]
                    self.state = 'MOVING_TO_GRASP_OBJECT'
                    if self.right_arm_ik(self.object_com):
                        destination = self.robotModel.getConfig()
                        motion.robot.right_mq.appendLinear(MOVE_TIME, Q_INTERMEDIATE_1)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                        self.state = 'MOVING_TO_GRASP_OBJECT'
                    else:
                        # TODO
                        time.sleep(50000)
                        print "Couldn't move there"
                elif self.state == 'MOVING_TO_GRASP_OBJECT':
                    if not motion.robot.right_mq.moving():
                        # Turn on vacuum, then move downwards to grasp
                        move_target = se3.apply(self.Tvacuum, [0, 0, 0])
                        move_target[2] = move_target[2] - GRASP_MOVE_DISTANCE
                        if self.right_arm_ik(move_target):
                            self.turnOnVacuum()
                            destination = self.robotModel.getConfig()
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                        else:
                            # TODO
                            time.sleep(50000)
                            print "Couldn't move there"
                        self.state = 'GRASPING_OBJECT'
                elif self.state == 'GRASPING_OBJECT':
                    self.wait_start_time = time.time()
                    self.state = 'WAITING_TO_GRASP_OBJECT'
                elif self.state == 'WAITING_TO_GRASP_OBJECT':
                    if time.time() - self.wait_start_time > GRASP_WAIT_TIME:
                        # Move back 
                        #move_target = se3.apply(self.Tvacuum, [0, 0, 0])
                        #move_target[0] = move_target[0] - BACK_UP_DISTANCE
                        if self.right_arm_ik(self.object_com):
                            destination = self.robotModel.getConfig()
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                        else:
                            # TODO
                            time.sleep(50000)
                            print "Couldn't move there"
                        motion.robot.right_mq.appendLinear(MOVE_TIME, Q_INTERMEDIATE_1)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, Q_INTERMEDIATE_2)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, Q_STOW)
                        self.state = 'MOVING_TO_STOW_OBJECT'
                elif self.state == 'MOVING_TO_STOW_OBJECT':
                    if not motion.robot.right_mq.moving():
                        self.state = 'STOWING_OBJECT'
                elif self.state == 'STOWING_OBJECT':
                        self.turnOffVacuum()
                        self.state = 'DONE'
                elif self.state == 'DONE':
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
    Trel = (so3.rotation((0,0,1),-math.pi/2),SHELF_MODEL_XFORM)
    T = world.rigidObject(0).getTransform()
    world.rigidObject(0).setTransform(*se3.mul(Trel,T))

    return world

def visualizerThreadFunction():
    visualizer.run()

if __name__ == '__main__':
    world = setupWorld()
    master = FullIntegrationMaster(world)
    visualizer = MyGLViewer(world, master)

    thread.start_new_thread(visualizerThreadFunction, ())
    master.start()
