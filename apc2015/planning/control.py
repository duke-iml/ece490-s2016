from api.control import ControlInterface
import logging, traceback
logger = logging.getLogger(__name__)

import numpy
from math import pi, ceil
import time

from integration import visualization

class RealControlInterface(ControlInterface):
    def resample_path(self, path, granularity=pi/50):
        path = numpy.array(path)

        fine_path = []
        for i in range(len(path)-1):
            # find the maximum distance in any joint coordinate
            steps = ceil(numpy.amax(path[i+1] - path[i]) / granularity)
            # interpolate the waypoint such that the largest joint change is small
            mini_path = numpy.array([ numpy.linspace(qi, qg, steps, False) for (qi, qg) in zip(path[i], path[i+1]) ]).T
            # add this to the plan
            fine_path.extend(mini_path)

        # add the final goal position
        fine_path.append(path[-1])

        logger.info('resampled path {} -> {}'.format(len(path), len(fine_path)))
        return list(fine_path)

    def execute(self, plan, sleep):
        '''
        Run the plan to completion or until an error occurs.  This method blocks
        until the plan execution is finished.

        sleep = method to call to perform a wait

        Returns True if the plan was run successfully.
        Returns False if an error occurred.
        '''
        
        # visualize all the paths starting with the robot's sensed config
        paths = [ p[1] for p in plan if p[0] in [ 'path', 'fast_path' ] ]
        if len(paths) > 0:
            visualization.debug_plan(self.knowledge_base, sum(paths, [ self.knowledge_base.robot_state.sensed_config ]))
        
        for (type, command, waitBefore, waitAfter) in plan:
            try:
                sleep(waitBefore)
                if(type == 'path'):
                    #visualization.debug_plan(self.knowledge_base, command)

                    logger.debug('Sending a plan')
                    self.robot.setMilestone(command[0])
                    for q in command[1:]:
                        # while(self.robot.isMoving()):
                            # sleep(.1)
                        self.robot.appendMilestone(q)
                    while(self.robot.isMoving()):
                        sleep(.1)
        #                 print 'waiting', self.robot.isMoving()
                elif(type == 'fast_path'):
                    visualization.debug_plan(self.knowledge_base, command)

                    logger.debug('sending a fast path')
                    self.robot.setConfig(command[0])
                    for q in command:
                        self.robot.appendConfig(q)                    
                    while(self.robot.isMoving()):
                        sleep(.1)
                elif(type == 'cartesian_drive'):
                    limb = command['limb']
                    error_threshold = command['error_threshold'] if 'error_threshold' in command else float('0.05')
                    velocity = command['velocity']
                    duration = command['duration']
                    angular_velocity = command['angularVelocity'] if 'angular_velocity' in command else [0,0,0]
                    print "Drive",velocity,"for time",duration
                    self.robot.setCartesianVelocityCommand(limb,velocity,angular_velocity)
                    t0 = time.time()
                    while(time.time() - t0 < duration):
                        #TODO: monitor errors
                        sleep(0.01)
                    self.robot.setCartesianVelocityCommand(limb,[0]*3,[0]*3)
                elif(type.endswith('gripper')):
                    logger.debug('Sending a gripper command')
                    if (type.startswith('left')):
                        pass
                        self.robot.commandGripper('left', command)
                    else:
                        pass
                        # self.robot.commandGripper('right', command)
                else:
                    logger.error('You messed up! Not a path or gripper command')
                    raise Exception

                sleep(waitAfter)

            except RuntimeError as e:
                print e 
                return False

        return True
