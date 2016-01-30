import logging, traceback
logger = logging.getLogger(__name__)

from multiprocessing import Process, Pipe
from klampt.robotsim import WorldModel

from apc.motion import FakeLowLevelController as LowLevelController

class RemoteRobotLost(Exception):
    pass

class RemoteRobot:
    def __init__(self, model_path):
        self.model_path = model_path

        methods = [
            'getSensedConfig',
            'getSensedVelocity',
            'getCommandedConfig',
            'getCommandedVelocity',
            'setPIDCommand',
            'setMilestone',
            'appendMilestone',
            'isMoving',
            'remainingTime',
            'commandGripper'
        ]

        # set up the interface
        for method in methods:
            # this outher wrapping function is necessary to capture the method variable
            def make_proxy(method):
                def proxy(*args, **kwargs):
                    return self._call(method, args, kwargs)
                return proxy

            setattr(self, method, make_proxy(method))

        self.child = None

    def start(self):
        if self.child:
            logger.warn('RemoteRobot is already started')
            return

        # create a pipe to communicate with the child
        (self.pipe, self.child_pipe) = Pipe()

        # create the subprocess
        self.child = Process(target=self._handler)
        # set the child to run as a background process (i.e., exit when parent does)
        self.child.daemon = True

        self.child.start()

    def close(self):
        if not self.child:
            logger.warn('RemoteRobot is already closed')
            return

        # this will signal the process to close
        self.pipe.close()
        # wait a bit for the process to exit
        self.child.join(1)

        # kill the process if it doesn't exit normally
        if self.child.is_alive():
            logger.warn('RemoteRobot escalting to kill child')
            self.child.terminate()
            # this join is needed to avoid zombie processes
            self.child.join()

        self.child = None

    def _call(self, method, args, kwargs):
        # check for active connection
        if not self.child:
            logger.error('RemoteRobot is not connected')
            raise RemoteRobotLost

        logger.debug('remote call {} {} {}'.format(method, args, kwargs))
        try:
            # perform the call
            self.pipe.send((method, args, kwargs))

            # wait for result
            while not self.pipe.poll(0.1):
                # check that subprocess is still alive
                if not self.child.is_alive():
                    logger.error('RemoteRobot died unexpectedly: {}'.format(self.child.exitcode))
                    # report the problem
                    raise RemoteRobotLost('unexpected child exit: {}'.format(self.child.exitcode))

            # get the result
            (output, error) = self.pipe.recv()
        except RemoteRobotLost:
            raise
        except Exception as e:
            # so the child process is in an undefined state now
            # this RemoteRobot needs to be destroyed
            logger.error('error during child communication: {}'.format(e))
            logger.error(traceback.format_exc())

            # clean up
            try:
                self.close()
            except:
                # last ditch effort to clean up
                self.child = None
                logger.warn('RemoteRobot error cleaning up after losing connection')

            # report the problem
            raise RemoteRobotLost(e)
        else:
            if not error:
                logger.debug('remote output: {}'.format(output))
                return output
            else:
                logger.debug('remote exception: {}'.format(error))
                raise error

    def _handler(self):
        logger.info('RemoteRobot {} started'.format(self.child.pid))

        # instantiate the robot
        model = WorldModel().loadRobot(self.model_path)
        logger.debug('robot model loaded from {}'.format(self.model_path))
        robot = LowLevelController(model)
        logger.debug('robot low-level controller initialized')

        # route remote calls
        while True:
            try:
                (method, args, kwargs) = self.child_pipe.recv()
            except EOFError:
                logger.info('child shutdown requested')
                # signal to shut down
                break

            try:
                # perform the call
                output = getattr(robot, method)(*args, **kwargs)
                logger.debug('child result: {}'.format(output))
            except Exception as e:
                # send the error
                self.child_pipe.send((None, e))
                logger.error('child exception: {}'.format(self.child.pid, e))
                logger.error(traceback.format_exc())
            else:
                # return the result
                self.child_pipe.send((output, None))

        # clean up
        self.child_pipe.close()
        # TODO: shutdown the robot
        #robot.motion.shutdown()

if __name__ == '__main__':
    import logging
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

    rr = RemoteRobot('klampt_models/baxter_with_parallel_gripper_col.rob')
    rr.start()

    rr.setMilestone(range(10))
    print rr.getSensedConfig()

    rr.close()
