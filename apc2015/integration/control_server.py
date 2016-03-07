import logging, traceback
logger = logging.getLogger(__name__)

import os
from time import sleep
from klampt.robotsim import WorldModel

from apc import baxter_scoop as baxter

if 'FAKE' in os.environ:
    logger.warn('using fake low-level controller by request')
    from apc.motion import FakeLowLevelController as LowLevelController
else:
    from apc.motion import PhysicalLowLevelController as LowLevelController
from interface import ControlInterface
from jobs import BaseJob, JobServer
from visualization import Viewer
from api.shared import RobotState

interval = 1.0/30

class ControlJob(BaseJob):
    def __init__(self, knowledge_base):
        # this knowledge base reference is used for updating the robot state
        self.knowledge_base = knowledge_base

        # create the viewer
        self.viewer = Viewer()

        BaseJob.__init__(self, None, None)

    def _get_interface(self):
        return ControlInterface

    def _handler(self):
        logger.debug('child {} start'.format(self.child.pid))

        # hold a reference to the world so that it is not garbage collected
        self.world = WorldModel()
        # create the robot model for the controller
        self.robot = self.world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
        logger.debug('robot model loaded from {}'.format(baxter.klampt_model_name))
        self.controller = LowLevelController(self.robot, baxter.klampt_model_name)
        logger.debug('controller low-level controller initialized')

        # send the initial controller state
        self.child_pipe.send((None, RobotState(self.controller)))

        while True:
            # get the call information
            try:
                # update the viewer while waiting for a command
                while not self.child_pipe.poll():
                    self._update_viewer()
                    sleep(interval)

                # receive the command
                (self.knowledge_base, self.method, self.args, self.kwargs) = self.child_pipe.recv()
            except EOFError:
                logger.info('child {} shutdown requested'.format(self.child.pid))
                # signal to shut down
                break

            # distinguish between pure knowledge base updates and method calls
            if not self.method:
                self._update_viewer()
                self._send_result(None)
                continue

            # route remote calls
            self.init_args = (self.controller, self.knowledge_base)
            # hook the sleep method to update the viewer during execution
            self.kwargs['sleep'] = lambda n: self._sleep(n)
            BaseJob._handler(self)

        # clean up
        # need to call the superclass since this class overrides with empty method
        BaseJob._close()
        # TODO: shutdown the controller
        #controller.motion.shutdown()

        self.viewer.close()

    def _sleep(self, n):
        while n > 0:
            self._update_viewer()
            n -= interval
            sleep(interval)

    def _update_viewer(self):
        try:
            # check to see if the viewer is alive
            if self.viewer.heartbeat:
                # update the viewer
                logger.info('updating viewer')
                self.viewer.update(self.knowledge_base, RobotState(self.controller))
                logger.info('done updating viewer')
            else:
                logger.info('viewer missed heartbeat')
                pass
        except Exception as e:
            logger.error('failed to update the viewer: {}'.format(e))

    def _send_result(self, result):
        # attach the robot state to the job result
        BaseJob._send_result(self, (result, RobotState(self.controller)))

    def _check(self):
        # run even if child is still alive
        # check for a result before attempting a blocking call
        if self.pipe.poll():
            self._process_result()
            self._done = True

            # extract the robot state and update the knowledge base
            if not self.error:
                (self._result, self.knowledge_base.robot_state) = self.result

        # check that the child is alive
        # this is done after the poll since the child could have sent a result and then executed
        if not self.alive:
            logger.error('child {} died unexpectedly: {}'.format(self.child.pid, self.child.exitcode))
            # set this exception as the error result
            self._error = Exception('child died unexpectedly: {}'.format(self.child.exitcode))
            self._done = True

    def _close(self):
        # keep the pipes open by default
        pass

class ControlServer:
    def __init__(self, knowledge_base):
        self.knowledge_base = knowledge_base
        self.job = None

    def start(self):
        if self.job:
            logger.warn('ControlServer is already started')
            return

        # start up the remote LowLevelController
        self.job = ControlJob(self.knowledge_base)
        self.job.run()
        logger.info('ControlServer starting')

        return self.job

    def safe_close(self):
        try:
            self.close()
        except Exception as e:
            logger.warn('ignoring error during safe close: {}'.format(e))
        finally:
            # guarantee this
            self.job = None

    def close(self):
        if not self.job:
            logger.warn('ControlServer is already closed')
            return

        # this will signal the process to close
        self.job.pipe.close()
        self.job.child_pipe.close()
        # wait a bit for the process to exit
        self.job.wait(1)

        # kill the process if it doesn't exit normally
        if self.job.alive:
            logger.warn('ControlServer escalating to kill child')
            self.job.cancel()

        self.job = None
        logger.info('ControlServer closed')

    def update(self):
        return self._call(None, None, None)

    def execute(self, *args, **kwargs):
        return self._call('execute', args, kwargs)

    def _call(self, method, args, kwargs):
        # check for active connection
        if not self.job:
            logger.error('ControlServer is not connected')
            return None

        # check that job has not died
        if not self.job.alive:
            logger.error('ControlServer child died unexpectedly: {}'.format(self.job.child.exitcode))
            # clean up
            self.safe_close()
            return None

        # reset all the flags (done/result/error flags in BaseJob class in jobs.py)
        self.job.clear()

        logger.debug('{} {} {}'.format(method, args, kwargs))
        try:
            # perform the call
            # need to send an updated copy of the knowledge base
            #
            self.job.pipe.send((self.knowledge_base, method, args, kwargs))
        except Exception as e:
            # so the child process is in an undefined state now
            # this ControlServer needs to be destroyed
            logger.error('error during child communication: {}'.format(e))
            logger.error(traceback.format_exc())

            self.safe_close()
            return None

        return self.job

    @property
    def running(self):
        return self.job is not None and self.job.alive

