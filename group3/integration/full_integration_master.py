import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

import json
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
import scipy.io as sio
import subprocess
import operator

baxter_rest_config = [0.0]*54

class FullIntegrationMaster:

    # Assorted functions
    # ==================
    def __init__(self, world):
        self.world = world
        self.current_bin = 'A'
        self.bin_state = {'A': {'done': False, 'contents': []},'B': {'done': False, 'contents': []}, 'C': {'done': False, 'contents': []}, 'D': {'done': False, 'contents': []}, 'E': {'done': False, 'contents': []}, 'F': {'done': False, 'contents': []}, 'G': {'done': False, 'contents': []}, 'H': {'done': False, 'contents': []}, 'I': {'done': False, 'contents': []}, 'J': {'done': False, 'contents': []}, 'K': {'done': False, 'contents': []}, 'L': {'done': False, 'contents': []}}
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
        self.calibratedCameraXform = RIGHT_F200_CAMERA_CALIBRATED_XFORM
        self.object_com = [0, 0, 0]
        self.points1 = []
        self.points2 = []

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
            self.turnOffVacuum()

        # Load JSON
        
        if DRY_RUN:
            with open(PICK_JSON_PATH) as pick_json_file:
                raw_json_data = json.load(pick_json_file)
            for k in self.bin_state:
                self.bin_state[k]['contents'] = raw_json_data['bin_contents']['bin_'+k]
            for my_dict in raw_json_data['work_order']:
                bin_letter = my_dict['bin'][4]
                self.bin_state[bin_letter]['target'] = my_dict['item']
            self.current_bin = planning.selectBin(self.bin_state)

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

        return self.world

    def load_real_robot_state(self):
        """Makes the robot model match the real robot"""
        self.robotModel.setConfig(motion.robot.getKlamptSensedPosition())

    def set_model_right_arm(self, q):
        destination = motion.robot.getKlamptSensedPosition()
        for index,v in enumerate(self.right_arm_indices):
            destination[v] = q[index]
        self.robotModel.setConfig(destination)

    def turnOnVacuum(self):
        if REAL_VACUUM:
            self.serial.write('H')
        else:
            print OKBLUE + "Fake vacuum is on" + END_COLOR

    def turnOffVacuum(self):
        if REAL_VACUUM:
            self.serial.write('L')
        else:
            print OKBLUE + "Fake vacuum is off" + END_COLOR

    def calibrateCamera(self):
        print self.calibratedCameraXform

        calibrateR = self.calibratedCameraXform[0];
        calibrateT = self.calibratedCameraXform[1];

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

            self.calibratedCameraXform = (calibrateR, calibrateT)

        except: 
            print "input error\n"

            print "printing camera calibration"
            print self.calibratedCameraXform

    def calibrateShelf(self):


        calibratedShelfXform = self.world.rigidObject(0).getTransform()
        print calibratedShelfXform
        calibrateR = calibratedShelfXform[0];
        calibrateT = calibratedShelfXform[1];

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
            self.world.rigidObject(0).setTransform(calibrateR, calibrateT)

 
            print self.world.rigidObject(0).getTransform()


        except: 
            print "input error\n"

            print "printing shelf calibration"
            print self.world.rigidObject(0).getTransform()


    # IK and motion planning
    # ======================
    def elbow_up(self):
        destination = self.robotModel.getConfig()
        for index,v in enumerate(self.right_arm_indices):
            if not (ELBOW_UP_BOUNDS[index][0] < destination[v] and destination[v] < ELBOW_UP_BOUNDS[index][1]):
                return False
        return True

    def right_arm_ik(self, right_target, ignore_elbow_up_constraint=True):
        """Solves IK to move the right arm to the specified
            right_target ([x, y, z] in world space)
        """
        self.load_real_robot_state()
        self.set_model_right_arm(eval('Q_IK_SEED_' + self.current_bin))

        qmin,qmax = self.robotModel.getJointLimits()
        for i in range(1000):
            point2_local = vectorops.add(VACUUM_POINT_XFORM[1], [.5, 0, 0])
            point2_world = vectorops.add(right_target, [0, 0, -.5])
            goal1 = ik.objective(self.robotModel.link('right_wrist'),local=VACUUM_POINT_XFORM[1],world=right_target)
            goal2 = ik.objective(self.robotModel.link('right_wrist'),local=point2_local,world=point2_world)
            if ik.solve([goal1, goal2],tol=0.0001) and (self.elbow_up() or ignore_elbow_up_constraint):
                return True
        print FAIL_COLOR + "right_arm_ik failed for " + str(right_target) + END_COLOR
        if not (self.elbow_up() or ignore_elbow_up_constraint):
            print FAIL_COLOR + str([self.robotModel.getConfig()[v] for v in self.right_arm_indices]) + END_COLOR
            print FAIL_COLOR + "IK found but elbow wasn't up" + END_COLOR
        return False

    def right_arm_ik_near_seed(self, right_target, ignore_elbow_up_constraint=True):
        """Solves IK to move the right arm to the specified
            right_target ([x, y, z] in world space)
        """
        self.load_real_robot_state()
        self.set_model_right_arm(eval('Q_IK_SEED_' + self.current_bin))

        oldRSquared = -1
        q_ik = None

        qmin,qmax = self.robotModel.getJointLimits()
        for i in range(1000):
            point2_local = vectorops.add(VACUUM_POINT_XFORM[1], [.5, 0, 0])
            point2_world = vectorops.add(right_target, [0, 0, -.5])
            goal1 = ik.objective(self.robotModel.link('right_wrist'),local=VACUUM_POINT_XFORM[1],world=right_target)
            goal2 = ik.objective(self.robotModel.link('right_wrist'),local=point2_local,world=point2_world)
            if ik.solve([goal1, goal2],tol=0.0001) and (self.elbow_up() or ignore_elbow_up_constraint):
                q_vals = [self.robotModel.getConfig()[v] for v in self.right_arm_indices];
                rSquared = vectorops.norm(vectorops.sub(q_vals, eval('Q_IK_SEED_' + self.current_bin)))
                if( oldRSquared <0 or oldRSquared > rSquared):
                    oldRSquared = rSquared
                    q_ik = q_vals 
        print FAIL_COLOR + "right_arm_ik failed for " + str(right_target) + END_COLOR
        if not (self.elbow_up() or ignore_elbow_up_constraint):
            print FAIL_COLOR + str([self.robotModel.getConfig()[v] for v in self.right_arm_indices]) + END_COLOR
            print FAIL_COLOR + "IK found but elbow wasn't up" + END_COLOR
        
        if(oldRSquared >= 0):
            self.set_model_right_arm(q_ik)
            return True
        return False

    # Main control loop
    # =================
    def loop(self):
        try:
            while True:
                print OKBLUE + "Bin " + str(self.current_bin) + ": " + self.state + END_COLOR

                if self.state == 'VISUAL_DEBUG':
                    self.object_com = [1.0839953170961105, -0.25145094946424207, 1.1241831909823194]
                    #self.set_model_right_arm( [0.1868786927065213, -1.567604604679142, 0.6922768776941961, 1.5862815343628953, -0.005567750307534711, -0.017979599494945674, 0.0035268645585939083])
                    self.load_real_robot_state()
                else:
                    self.load_real_robot_state()

                self.Tcamera = se3.mul(self.robotModel.link('right_lower_forearm').getTransform(), self.calibratedCameraXform)
                self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)

                self.vacuumPc = Geometry3D()
                self.vacuumPc.loadFile(VACUUM_PCD_FILE)
                temp_xform = self.robotModel.link('right_wrist').getTransform()
                self.vacuumPc.transform(self.Tvacuum[0], self.Tvacuum[1])

                if self.state == 'DEBUG_COLLISION_CHECKER':
                    temp_planner = planning.LimbPlanner(self.world, self.vacuumPc)
                    print 'Is this configuration collision free?'
                    print temp_planner.check_collision_free('right')
                    sys.stdout.flush()

                elif self.state == 'FAKE_PATH_PLANNING':
                    self.object_com = [1.114, -.077, 1.412]
                    self.right_arm_ik(self.object_com)
                    self.Tcamera = se3.mul(self.robotModel.link('right_lower_forearm').getTransform(), self.calibratedCameraXform)
                    self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)
                    time.sleep(2342340)

                elif self.state == 'START':
                    self.state = "MOVE_TO_SCAN_BIN"

                elif self.state == 'MOVE_TO_SCAN_BIN':
                    for milestone in eval('Q_BEFORE_SCAN_' + self.current_bin):
                        print "Moving to " + str(milestone)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(milestone))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)
                    motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_SCAN_BIN_' + self.current_bin)))
                    self.state = 'MOVING_TO_SCAN_BIN'

                elif self.state == 'MOVING_TO_SCAN_BIN':
                    if not motion.robot.right_mq.moving():
                        self.wait_start_time = time.time()
                        self.state = 'WAITING_TO_SCAN_BIN'

                elif self.state == 'WAITING_TO_SCAN_BIN':
                    if time.time() - self.wait_start_time > SCAN_WAIT_TIME:
                        if REAL_PERCEPTION:
                            self.state = 'SCANNING_BIN'
                        else:
                            self.state = 'FAKE_SCANNING_BIN'

                elif self.state == 'SCANNING_BIN':
                    print "Waiting for message from camera"    
                    cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
                   
                    if perception.isCloudValid(cloud):
                        cloud = perception.convertPc2ToNp(cloud)
                        np_cloud = cloud[::STEP]

                        self.points1 = []
                        for point in np_cloud:
                            transformed = se3.apply(self.Tcamera, point)
                            self.points1.append(transformed)

                        np_cloud = perception.subtractShelf(np_cloud, self.current_bin)
                    
                        self.points2 = []
                        for point in np_cloud:
                            transformed = se3.apply(self.Tcamera, point)
                            self.points2.append(transformed)

                        self.object_com = se3.apply(self.Tcamera, perception.com(np_cloud))

                        if PRINT_BLOBS:
                            print np_cloud

                        sio.savemat(CLOUD_MAT_PATH, {'cloud':np_cloud})
                        fo = open(CHENYU_GO_PATH, "w")
                        fo.write("chenyugo")
                        fo.close()

                        self.object_com = se3.apply(self.Tcamera, perception.com(np_cloud))

                        if CALIBRATE_CAMERA:
                            self.state = "CALIBRATE_CAMERA"
                        elif SEGMENT:
                            self.state = "WAITING_FOR_SEGMENTATION"
                        else:
                            self.state = "MOVE_TO_GRASP_OBJECT"
                    else:
                        print FAIL_COLOR + "Got an invalid cloud, trying again" + END_COLOR

                elif self.state == 'FAKE_SCANNING_BIN':
                    self.object_com = [1.1114839836097854, -0.6087936130127559, 0.9899267043340634]

                    if DRY_RUN:
                        raw_input("Hit enter to continue: ")

                    self.state = 'MOVE_TO_GRASP_OBJECT'

                elif self.state == 'CALIBRATE_CAMERA':
                    self.calibrateCamera()
                    self.state = 'SCANNING_BIN'

                elif self.state == 'CALIBRATE_SHELF':
                    self.calibrateShelf()
                    #make blocking

                elif self.state == 'WAITING_FOR_SEGMENTATION':
                    if os.path.isfile(CHENYU_DONE_PATH):
                        os.remove(CHENYU_DONE_PATH)
                        self.state = 'POSTPROCESS_SEGMENTATION'

                elif self.state == 'POSTPROCESS_SEGMENTATION':
                    object_blobs = []
                    time.sleep(5)
                    for i in range(1,20):
                        seg_file_path = MAT_PATH + "seg" + str(i) + ".mat"
                        print seg_file_path
                        if os.path.isfile(seg_file_path):
                            print seg_file_path + " exists"
                            mat_contents = sio.loadmat(seg_file_path)
                            r = mat_contents['r']
                            r = r[r[:,1]!=0, :]
                            object_blobs.append(r)
                            os.remove(seg_file_path)
                    if PRINT_BLOBS:
                        print "============="
                        print "object blobs"
                        print object_blobs
                        print "============="

                    if len(object_blobs) == 0:
                        self.state = 'BIN_DONE'
                    else:
                        object_list = [ITEM_NUMBERS[item_str] for item_str in self.bin_state[self.current_bin]['contents']]
                        target = ITEM_NUMBERS[self.bin_state[self.current_bin]['target']]

                        histogram_dict = perception.loadHistogram(object_list)
                        cloud_label = {} # key is the label of object, value is cloud points
                        label_score = {} # key is the label, value is the current score for the object 
                        for object_cloud in object_blobs:
                            if PRINT_BLOBS:
                                print "====================="
                                print 'object_cloud'
                                print object_cloud
                                print '====================='
                            object_cloud = perception.resample(cloud,object_cloud,3)
                            label,score = perception.objectMatch(object_cloud,histogram_dict)
                            if label in cloud_label:
                                if label_score[label] < score:
                                    label_score[label] = score
                                    cloud_label[label] = object_cloud
                            else:
                                cloud_label[label] = object_cloud
                                label_score[label] = score

                        if PRINT_LABELS_AND_SCORES:
                            print cloud_label
                            print "============="
                            print label_score
                        if target in cloud_label:
                            self.object_com = se3.apply(self.Tcamera, perception.com(cloud_label[target]))

                            self.points1 = []
                            for point in cloud_label[target]:
                                transformed = se3.apply(self.Tcamera, point)
                                self.points1.append(transformed)

                            self.points2 = []
                        else:
                            cloud_score = {}
                            histogram_dict = perception.loadHistogram([target])
                            for object_cloud in object_blobs:
                                object_cloud = perception.resample(cloud,object_cloud,3)
                                label,score = perception.objectMatch(object_cloud,histogram_dict)
                                cloud_score[score] = object_cloud
                            sorted_cloud = sorted(cloud_score.items(), key=operator.itemgetter(0),reverse = True)
                            score  = sorted_cloud[0][0]
                            com = perception.com(sorted_cloud[0][1])
                            self.points1 = []
                            self.points2 = []
                            for point in sorted_cloud[0][1]:
                                transformed = se3.apply(self.Tcamera, point)
                                self.points1.append(transformed)
                            self.object_com = se3.apply(self.Tcamera, com)
                        self.state = 'MOVE_TO_GRASP_OBJECT'

                elif self.state == 'MOVE_TO_GRASP_OBJECT':
                    for milestone in eval('Q_AFTER_SCAN_' + self.current_bin):
                        print "Moving to " + str(milestone)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(milestone))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)

                    motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                    while motion.robot.right_mq.moving():
                        time.sleep(1)
                    time.sleep(1)

                    self.load_real_robot_state()
                    self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)

                    print WARNING_COLOR + str(self.object_com) + END_COLOR
                    self.object_com = vectorops.add(self.object_com, COM_ADJUSTMENT)

                    current_vacuum_point = se3.apply(self.Tvacuum, [0, 0, 0])
                    milestone = vectorops.add(current_vacuum_point, self.object_com)
                    milestone = vectorops.div(milestone, 2)
                    
                    if DRY_RUN:
                        self.state = 'MOVING_TO_GRASP_OBJECT'
                    else:
                        if self.right_arm_ik(milestone):
                            destination = self.robotModel.getConfig()
                            self.q_milestone = [destination[v] for v in self.right_arm_indices]
                            print WARNING_COLOR + "IK config for " + str(milestone) + ": " + str(self.q_milestone) + END_COLOR
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(self.q_milestone))
                        else:
                            print FAIL_COLOR + "Error: IK failed" + END_COLOR
                            sys.stdout.flush()
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                            while motion.robot.right_mq.moving():
                                time.sleep(1)
                            time.sleep(1)

                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)

                        if self.right_arm_ik(self.object_com):
                            destination = self.robotModel.getConfig()
                            print WARNING_COLOR + "IK config for " + str(self.object_com) + ": " + str([destination[v] for v in self.right_arm_indices]) + END_COLOR
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                            self.state = 'MOVING_TO_GRASP_OBJECT'
                        else:
                            print FAIL_COLOR + "Error: IK failed" + END_COLOR
                            sys.stdout.flush()
                            motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                            while motion.robot.right_mq.moving():
                                time.sleep(1)
                            time.sleep(1)

                elif self.state == 'MOVING_TO_GRASP_OBJECT':
                    if not motion.robot.right_mq.moving():
                        time.sleep(1)
                        self.state = 'GRASP_OBJECT'

                elif self.state == 'GRASP_OBJECT':
                    move_target = se3.apply(self.Tvacuum, [0, 0, 0])
                    move_target[2] = move_target[2] - GRASP_MOVE_DISTANCE - (MEAN_OBJ_HEIGHT / 2)
                    if self.right_arm_ik(move_target):
                        self.turnOnVacuum()
                        destination = self.robotModel.getConfig()
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                    else:
                        print FAIL_COLOR + "Error: IK failed" + END_COLOR
                        sys.stdout.flush()
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)
                    self.wait_start_time = time.time()
                    self.state = 'WAITING_TO_GRASP_OBJECT'

                elif self.state == 'WAITING_TO_GRASP_OBJECT':
                    if time.time() - self.wait_start_time > GRASP_WAIT_TIME:
                        self.state = 'MOVE_UP_BEFORE_RETRACT'

                elif self.state == 'MOVE_UP_BEFORE_RETRACT':
                    move_target = se3.apply(self.Tvacuum, [0, 0, 0])
                    move_target[2] = move_target[2] + GRASP_MOVE_DISTANCE + (MEAN_OBJ_HEIGHT / 2)
                    if self.right_arm_ik(move_target):
                        self.turnOnVacuum()
                        destination = self.robotModel.getConfig()
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig([destination[v] for v in self.right_arm_indices]))
                    else:
                        print FAIL_COLOR + "Error: IK failed" + END_COLOR
                        sys.stdout.flush()
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)
                    self.state = 'MOVE_TO_STOW_OBJECT'

                elif self.state == 'MOVE_TO_STOW_OBJECT':
                    motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(eval('Q_IK_SEED_' + self.current_bin)))
                    while motion.robot.right_mq.moving():
                        time.sleep(1)
                    time.sleep(1)

                    if not DRY_RUN:
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(self.q_milestone))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)

                    for milestone in eval('Q_AFTER_GRASP_' + self.current_bin):
                        print "Moving to " + str(milestone)
                        motion.robot.right_mq.appendLinear(MOVE_TIME, planning.cleanJointConfig(milestone))
                        while motion.robot.right_mq.moving():
                            time.sleep(1)
                        time.sleep(1)
                    motion.robot.right_mq.appendLinear(MOVE_TIME, Q_STOW)
                    self.state = 'MOVING_TO_STOW_OBJECT'

                elif self.state == 'MOVING_TO_STOW_OBJECT':
                    if not motion.robot.right_mq.moving():
                        self.state = 'STOWING_OBJECT'

                elif self.state == 'STOWING_OBJECT':
                        self.turnOffVacuum()
                        self.wait_start_time = time.time()
                        self.state = 'WAITING_FOR_SECURE_STOW'

                elif self.state == 'WAITING_FOR_SECURE_STOW':
                    if time.time() - self.wait_start_time > GRASP_WAIT_TIME:
                        self.state = 'BIN_DONE' if SELECT_REAL_BIN else 'DONE'

                elif self.state == 'BIN_DONE':
                    self.bin_state[self.current_bin]['done'] = True
                    self.current_bin = planning.selectBin(self.bin_state)
                    if self.current_bin is None:
                        self.state = 'DONE'
                    else:
                        self.state = 'START'

                elif self.state == 'DONE':
                    print "actual vacuum point: ", se3.apply(self.Tvacuum, [0, 0, 0])

                else:
                    print FAIL_COLOR + "Unknown state" + END_COLOR


                time.sleep(1)
        except KeyboardInterrupt:
            motion.robot.stopMotion()
            sys.exit(0)

def setupWorld():
    world = WorldModel()
    print "Loading full Baxter model (be patient, this will take a minute)..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"baxter.rob"))
    #print "Loading simplified Baxter model..."
    #world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"kiva_pod/model.obj"))
    print "Loading plane model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"plane.env"))



    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    
    #Trel = (so3.rotation((0,0,1),-math.pi/2), SHELF_MODEL_XFORM)
    Trel = SHELF_MODEL_XFORM_CALIBRATED
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
