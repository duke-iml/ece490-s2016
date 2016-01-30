import logging, traceback
logger = logging.getLogger(__name__)

from multiprocessing import Process, Pipe
import sys, os, signal
from time import time
import pickle

from OpenGL.GL import glEnable, glDisable, glBlendFunc, glMaterialfv
from OpenGL.GL import glBegin, glEnd, glNewList, glEndList, glGenLists, glCallList, glVertex3f, glColor3f, glPointSize
from OpenGL.GL import GL_BLEND, GL_SRC_ALPHA, GL_FRONT_AND_BACK, GL_ONE_MINUS_SRC_ALPHA, GL_AMBIENT_AND_DIFFUSE, GL_COMPILE, GL_POINTS, GL_LIGHTING
from OpenGL.GLUT import glutLeaveMainLoop, glutPostRedisplay, glutPositionWindow
from klampt.glprogram import GLRealtimeProgram
from klampt.robotsim import WorldModel
from klampt import gldraw, so3, se3, vectorops

from apc import baxter_scoop as baxter

def debug_plan(world, plan, wait=True):
    if 'NO_VIZ_PLAN' not in os.environ:
        _debug(lambda: PlanViewer(world, plan), wait)

def debug_world(world, xforms=None, wait=True):
    if 'NO_VIZ_WORLD' not in os.environ:
        _debug(lambda: WorldViewer(world, xforms), wait)
debug = debug_world

def debug_cloud(clouds, xforms=None, world=None, wait=True):
    if 'NO_VIZ_CLOUD' not in os.environ:
        _debug(lambda: CloudViewer(clouds, xforms, world), wait)

def _debug(factory, wait=True):
    if 'NO_VIZ' in os.environ:
        return

    pid = os.fork()
    if not pid:
        # child process
        logger.debug('child started')
        # run the viewer
        viewer = factory()
        viewer.run()
        logger.debug('child exiting')
        # cleanup without trashing parent filenos
        os._exit(0)
    else:
        # parent process
        if wait:
            logger.debug('parent waiting for child {}'.format(pid))
            # wait for child to finish
            try:
                os.waitpid(pid, 0)
                logger.debug('parent wait done')
            except OSError as e:
                logger.error('error during wait for child {}: {}'.format(pid, e))
                logger.error(traceback.format_exc())
                # kill the child now
                logger.debug('escalating to kill child')
                os.kill(pid, signal.SIGINT)
        else:
            logger.debug('parent spawned child {}'.format(pid))

class WorldViewer(GLRealtimeProgram):
    def __init__(self, world, xforms=None, title=None):
        GLRealtimeProgram.__init__(self, title or 'World Visualization')

        self.world = world
        self.xforms = xforms or []
        self.xforms += [ se3.identity() ]

        # configure the rendering
        self.clippingplanes = (0.1, 100)

        logger.info('visualization ready')

    def initialize(self):
        glutPositionWindow(0, 600)
        GLRealtimeProgram.initialize(self)

    def display(self):
        self.world.drawGL()

        for xform in self.xforms:
            gldraw.xform_widget(xform, 0.1, 0.01)

    def keyboardfunc(self,c,x,y):
        if c == 'q' or c == '\x1b':
            os._exit(0)
            logger.info('visualization exiting')

def debug_cloud(clouds, xforms=None, world=None, wait=True):
    _debug(lambda: CloudViewer(clouds, xforms, world), wait)

def _debug(factory, wait=True):
    pid = os.fork()
    if not pid:
        # child process
        logger.debug('child started')
        # run the viewer
        viewer = factory()
        viewer.run()
        logger.debug('child exiting')
        # cleanup without trashing parent filenos
        os._exit(0)
    else:
        # parent process
        if wait:
            logger.debug('parent waiting for child {}'.format(pid))
            # wait for child to finish
            try:
                os.waitpid(pid, 0)
                logger.debug('parent wait done')
            except OSError as e:
                logger.error('error during wait for child {}: {}'.format(pid, e))
                logger.error(traceback.format_exc())
                # kill the child now
                logger.debug('escalating to kill child')
                os.kill(pid, signal.SIGINT)
        else:
            logger.debug('parent spawned child {}'.format(pid))

class CloudViewer(GLRealtimeProgram):
    def __init__(self, clouds, xforms=None, world=None):
        GLRealtimeProgram.__init__(self, 'Cloud Visualization')

        self.clouds = clouds or []
        self.point_lists = []

        self.xforms = xforms or []
        self.xforms += [ se3.identity() ]

        self.world = world

        self.point_size = 2

        # configure the rendering
        self.clippingplanes = (0.1, 100)

        logger.info('visualization ready')

    def initialize(self):
        glutPositionWindow(0, 600)
        GLRealtimeProgram.initialize(self)

        colors = [ (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (0, 1, 1), (1, 0, 1) ]

        for (i, cloud) in enumerate(self.clouds):
            point_list = glGenLists(i + 1)
            self.point_lists.append(point_list)

            # compile the point cloud
            glNewList(point_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            glBegin(GL_POINTS)
            for point in cloud:
                if len(point) == 2:
                    xyz = point[0]
                    rgb = point[1]
                else:
                    xyz = point[:3]
                    if len(point) == 4:
                        rgb = point[3]
                    elif len(point) > 4:
                        rgb = point[3:6]
                    else:
                        rgb = None

                if rgb is not None:
                    glColor3f(*map(lambda x: x/255.0, rgb))
                else:
                    glColor3f(*colors[i % len(colors)])
                glVertex3f(*xyz[:3])
            glEnd()
            glEndList()
            logging.debug('compiled {} points for cloud {}'.format(len(cloud), i))

    def display(self):
        if self.world:
            self.world.drawGL()

        glPointSize(max(1, self.point_size))
        for point_list in self.point_lists:
            glCallList(point_list)

        for xform in self.xforms:
            gldraw.xform_widget(xform, 0.1, 0.01)

    def keyboardfunc(self,c,x,y):
        if c == '+':
            self.point_size += 1
        elif c == '-':
            self.point_size -= 1
        elif c == 'q' or c == '\x1b':
            os._exit(0)
            logger.info('visualization exiting')

class KnowledgeBaseViewer(WorldViewer):
    def __init__(self, knowledge_base=None, title=None):
        world = WorldModel()
#         self.ground = world.loadTerrain('klampt_models/plane.env')
        self.shelf = None
        self.order_bin = None
        self.objects = {}
        self.n = 0

        # instantiate the robot
        self.robot = world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
        logger.debug('robot model loaded from {}'.format(baxter.klampt_model_name))

        self.knowledge_base = knowledge_base
        self.robot_state = None

        WorldViewer.__init__(self, world, title=title or 'Knowledge Base Viewer')

    def display(self):
        if self.knowledge_base:
            # update the world
            if self.knowledge_base.shelf_xform:
                # load the shelf once a transform is available
                if not self.shelf:
                    self.shelf = self.world.loadRigidObject('klampt_models/north_shelf/shelf_with_bins.obj')
                    logger.info('spawned shelf model')
                self.shelf.setTransform(*self.knowledge_base.shelf_xform)

            if self.knowledge_base.order_bin_xform:
                # load the order bin once a transform is available
                if not self.order_bin:
                    self.order_bin = self.world.loadRigidObject('klampt_models/apc_bin/apc_bin.obj')
                    logger.info('spawned order bin model')
                self.order_bin.setTransform(*self.knowledge_base.order_bin_xform)

            # spawn/update objects
            for (name, xform) in self.knowledge_base.object_xforms.items():
                # load the object once a transform is availale
                if name not in self.objects:
                    body = self.world.loadRigidObject('klampt_models/items/{0}/{0}.obj'.format(name))
                    logger.info('spawned {} model'.format(name))
                
                    # load the point cloud
                    if name in self.knowledge_base.object_clouds:                    
                        self.n += 1
                        display_list = glGenLists(self.n)

                        # compile the display list
                        glNewList(display_list, GL_COMPILE)
                        glDisable(GL_LIGHTING)
                        glBegin(GL_POINTS)
                        points = self.knowledge_base.object_clouds[name]
                        for point in points:
                            if len(point) == 2:
                                xyz = point[0]
                                rgb = point[1]
                            else:
                                xyz = point[:3]
                                if len(point) == 4:
                                    rgb = point[3]
                                elif len(point) > 3:
                                    rgb = point[3:6]
                                else:
                                    rgb = None

                            if rgb is not None:
                                glColor3f(*map(lambda x: x/255.0, rgb))
                            else:
                                glColor3f(*colors[i % len(colors)])
                            glVertex3f(*xyz[:3])
                        glEnd()
                        glEndList()
                        logging.debug('compiled {} points for {}'.format(len(points), name))
                    else:
                        display_list = None

                    self.objects[name] = {
                        'body': body,
                        'display_list': display_list
                    }
                    
                #self.objects[name]['body'].setTransform(*xform)

            # delete objects
            for (name, props) in self.objects.items():
                if name not in self.knowledge_base.object_xforms:
                    # remove the object
                    # XXX: cannot actually delete object... so move it far away... very far away
                    props['body'].setTransform(so3.identity(), [ 1e3, 1e3, 1e3 ])
                    del self.objects[name]

        # update the robot state
        if self.robot_state:
            try:
                self.robot.setConfig(self.robot_state.sensed_config)
            except TypeError as e:
                logger.error('error visualizing config: {}'.format(self.robot_state.sensed_config))
                logger.error(traceback.format_exc())
                sys.exit(-1)

        self.world.drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])

        if self.robot_state:
            qi = self.robot.getConfig()
            self.robot.setConfig(self.robot_state.commanded_config)
            self.robot.drawGL(False)
            self.robot.setConfig(qi)

        glDisable(GL_BLEND)

        # draw the point clouds
        glPointSize(2)
        for (name, props) in self.objects.items():
            if 'display_list' in props:
                glCallList(props['display_list'])

        # draw gripper link transforms     
        for name in [ 'left_gripper', 'right_gripper' ]:
            gldraw.xform_widget(self.robot.getLink(name).getTransform(), 0.1, 0.01)
        # draw axis-aligned axes for reference
        gldraw.xform_widget((so3.identity(), [0, 0, 1]), 0.1, 0.01)
        # draw local origins of objects
        if self.knowledge_base:
            for xform in self.knowledge_base.object_xforms.values():
                gldraw.xform_widget(xform, 0.1, 0.01)


class PlanViewer(KnowledgeBaseViewer):
    def __init__(self, knowledge_base, plan, title=None):
        self.plan = plan or []
        if not self.plan:
            logger.warn('received empty plan for visualization')

        self.running = True
        self.direction = 1
        self.delay = 1.0/2
        self.active_index = 0
        self.time_mark = time()

        KnowledgeBaseViewer.__init__(self, knowledge_base, title=title or 'Plan Visualization')
        self.robot_state = self.knowledge_base.robot_state

    def idle(self):
        if self.running:
            if time() - self.time_mark >= self.delay:
                self.active_index += self.direction
                self.time_mark = time()

                if self.active_index <= 0:
                    #self.direction = 1
                    self.active_index = 0
                #elif self.active_index >= len(self.plan)-1:
                #    #self.direction = -1
                #    self.active_index = len(self.plan)-1
                elif self.active_index >= len(self.plan):
                    self.active_index = 0
                    

                logger.info('config {} of {}'.format(self.active_index, len(self.plan)-1))

        if self.plan:     
            u = (time() - self.time_mark)/self.delay
            if self.active_index + 1 < len(self.plan):
                self.knowledge_base.robot_state.sensed_config = vectorops.interpolate(self.plan[self.active_index],self.plan[self.active_index+1],u)
            else: 
                self.knowledge_base.robot_state.sensed_config = self.plan[self.active_index]
        glutPostRedisplay()

    def keyboardfunc(self,c,x,y):
        if c == 'g':
            self.running = not self.running
            logger.info('running: {}'.format(self.running))

        elif c == 'n':
            if not self.running:
                self.active_index = max(0, self.active_index-1)
                logger.info('config {} of {}'.format(self.active_index, len(self.plan)-1))
        elif c == 'm':
            if not self.running:
                self.active_index = min(self.active_index+1, len(self.plan)-1)
                logger.info('config {} of {}'.format(self.active_index, len(self.plan)-1))
        else:
            WorldViewer.keyboardfunc(self, c, x, y)


class RemoteKnowledgeBaseViewer(KnowledgeBaseViewer):
    def __init__(self, child_pipe):
        self.child_pipe = child_pipe
        # send heartbeat
        self.child_pipe.send(True)

        KnowledgeBaseViewer.__init__(self)

    def idle(self):
        # check for updated state
        if self.child_pipe.poll():
            try:
                #path = self.child_pipe.recv()
                #logger.error('remote path: {}'.format(path))
                #(self.knowledge_base, self.robot_state) = pickle.load(open(path))
                #logger.error('finished loading')
                (self.knowledge_base, self.robot_state) = self.child_pipe.recv()
            except EOFError:
                logger.info('visualization exiting from EOF')
                sys.exit(0)
            else:
                # acknowledge the receipt
                self.child_pipe.send(True)

            glutPostRedisplay()

    def keyboardfunc(self,c,x,y):
        if c == 'x' or c == '\x1b':
            logger.info('visualization exiting from keypress')
            os._exit(0)

class Viewer:
    def __init__(self):
        # create a pipe to communicate with the child
        (self.pipe, self.child_pipe) = Pipe()

        # create the subprocess
        self.child = Process(target=self._handler)
        # set the child to run as a background process (i.e., exit when parent does)
        self.child.daemon = True

        self.child.start()

    def update(self, knowledge_base, robot_state):
        self.pipe.send((knowledge_base, robot_state))
        #path = '/tmp/transfer.pickle'
        #pickle.dump((knowledge_base, robot_state), open(path, 'wb'))
        #self.pipe.send(path)

    def close(self):
        # signal the child to close
        self.pipe.close()
         # wait a bit for the process to exit
        self.child.wait(1)

        # kill the process if it doesn't exit normally
        if self.child.is_alive():
            logger.warn('Viewer escalating to kill child')
            self.child.cancel()

        self.child = None
        logger.info('Viewer closed')

    def _handler(self):
        logger.debug('child {} start'.format(self.child.pid))

        viewer = RemoteKnowledgeBaseViewer(self.child_pipe)
        viewer.run()

    @property
    def heartbeat(self):
        logger.info('checking for heartbeat')
        if self.pipe.poll():
            logger.info('heartbeat present')
            self.pipe.recv()
            logger.info('heartbeat read')
            return True
        else:
            return False
