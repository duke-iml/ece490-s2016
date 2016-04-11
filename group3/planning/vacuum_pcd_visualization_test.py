import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

import random
import time
import serial
import thread
from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from klampt.robotsim import *
from OpenGL.GL import *
from planning.MyGLViewer import MyGLViewer
from util.constants import *

baxter_rest_config = [0.0]*54

class VacuumPcdMaster:
    def __init__(self, world):
        self.world = world
        self.robotModel = world.robot(0)
        self.state = INITIAL_STATE
        # self.state = 'SCANNING_BIN'
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


        self.geo = Geometry3D()
        self.geo.loadFile('custom_vacuum.pcd')

        # Set up serial
        if REAL_VACUUM:
            self.serial = serial.Serial()
            self.serial.port = ARDUINO_SERIAL_PORT
            self.serial.baudrate = 9600
            self.serial.open()
            if self.serial.isOpen():
                self.serial.write("hello")
                response = self.serial.read(self.serial.inWaiting())

    def drawStuff(self):
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glColor3f(1.0,1.0,1.0)
        glBegin(GL_POINTS)
        for i in range(self.geo.getPointCloud().numPoints()):
            point = self.geo.getPointCloud().getPoint(i)
            glVertex3f(point[0], point[1], point[2])
        glEnd()
        

    def start(self):
        self.loop()

    def loop(self):
        while True:
            self.Tvacuum = se3.mul(self.robotModel.link('right_wrist').getTransform(), VACUUM_POINT_XFORM)
            self.geo = Geometry3D()
            self.geo.loadFile('custom_vacuum.pcd')
            self.geo.transform(self.Tvacuum[0], self.Tvacuum[1])
            time.sleep(1)

def setupWorld():
    world = WorldModel()
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"))
    #print "Loading Kiva pod model..."
    #world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"kiva_pod/model.obj"))
    print "Loading plane model..."
    world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
    #T = world.rigidObject(0).getTransform()
    #world.rigidObject(0).setTransform(*se3.mul(Trel,T))

    return world

def visualizerThreadFunction():
    visualizer.run()

if __name__ == '__main__':
    world = setupWorld()
    master = VacuumPcdMaster(world)
    visualizer = MyGLViewer(world, master)

    thread.start_new_thread(visualizerThreadFunction, ())
    master.start()
