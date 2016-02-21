import logging, traceback
logger = logging.getLogger(__name__)

from time import sleep
import os, json, csv

from planning_server import PlanningServer
from control_server import ControlServer
from perception_server import PerceptionServer
from jobs import JobManager
from apc import baxter_scoop as baxter
import copy

from api.shared import KnowledgeBase

knowledge_base_dir = 'kb/'

#dictionary mapping Amazon names to the Berkeley names
translation_to_berkeley = { 'kyjen_squeakin_eggs_plush_puppies':'kygen_squeakin_eggs_plush_puppies',
                            'rolodex_jumbo_pencil_cup':'rollodex_mesh_collection_jumbo_pencil_cup',
                            'first_years_take_and_toss_straw_cup':'first_years_take_and_toss_straw_cups'}

def wait_task(task, done, timeout=30, delay=0.5):
    total_wait = 0
    while not done() and total_wait < timeout:
        sleep(delay)
        total_wait += delay

    if not done():
        task.cancel()
        logging.warn('task canceled early with timeout of {} seconds'.format(timeout))
        return False
    else:
        return True

# TODO: Understand / Go through this MotionManager class. (Especially self._call method)
class MotionManager:
    def __init__(self, knowledge_base, **params):
        self.knowledge_base = knowledge_base
        params = params or {}

        self.planner = PlanningServer(knowledge_base, params.get('parallelism', 5))
        self.control = ControlServer(knowledge_base)
        self.pre_execute_robot_state = None
        self.last_executed_plan = None

        methods = [
            'moveToVantagePoint',
            'graspObjectInBin',
            'moveObjectToOrderBin',
            'moveToInitialPose',
            'moveToXform',
            'PickUpOrderTray',
            'MoveTrayToBin',
            'MoveTrayToOrderBin'
        ]

        # set up the interface so that calling "MotionManager.methodName" like
        # "MotionManager.moveToVantagePoint" or "self.manager.moveToVantagePoint" inside Master class
        # invokes the "MotionManager._call(method, args ,kwargs)" function
        for method in methods:
            # method contains a string of methodName

            # this outher wrapping function is necessary to capture the method variable
            def make_proxy(method):
                # see here for *args and **kwargs
                # http://stackoverflow.com/questions/3394835/args-and-kwargs
                def proxy(*args, **kwargs):
                    return self._call(method, args, kwargs)

                # returns the function handle "self._call(method, args, kwargs)"
                return proxy

            # equivalent to self.method = make_proxy(method)
            #                           = self._call(method, *args, **kwargs)
            # ie.) self.moveToVantagePoint = function_handle of self._call(method, args, kwargs)
            setattr(self, method, make_proxy(method))

    def start(self):
        logger.info('starting control server')
        task = self.control.start()

        # check that control server started
        wait_task(task, lambda: task.done, 10)
        if task.error:
            logger.error('failed to start control server')
            return False
        else:
            logger.info('control server started')
            return True

        self.pre_execute_robot_state = None
        self.last_executed_path = None

    def _check_control(self):
        if self.control.running:
            return True
        else:
            logger.error('control server has died -> restarting')
            self._restart_control()
            return False

    def _restart_control(self):
        self.control.safe_close()
        self.start()

    def undo_last_motion(self):
        if not self.last_executed_path:
            logger.error('Trying to undo last motion, but no motion was previously executed')
            return False
        #get the gripper commands leading up to the end of the motion
        previous_gripper_commands = [baxter.get_q_gripper_command(self.pre_execute_robot_state.commanded_config,'left')]
        for (type, command, waitBefore, waitAfter) in self.last_executed_path:
            if type.endswith('gripper'):
                if (type.startswith('left')):
                    previous_gripper_commands.append(command)
        #go through the previous path
        reverse_path = list(reversed(self.last_executed_path))
        ngripper_commands = 0
        for index,(type, command, waitBefore, waitAfter) in enumerate(reverse_path):
            if(type == 'path' or type == 'fast_path'):
                #reverse the path
                command = command[::-1]
            elif(type.endswith('gripper')):
                #go to the prior gripper config
                command = previous_gripper_commands[-2-ngripper_commands]
                if (type.startswith('left')):
                    ngripper_commands += 1
            else:
                logger.error('You messed up! Not a path or gripper command')
                raise Exception
            reverse_path [index] = type,command,waitAfter,waitBefore

        self.pre_execute_robot_state = copy.deepcopy(self.knowledge_base.robot_state)
        self.last_executed_path = reverse_path

        task = self.control.execute(reverse_path)
        # check that the plan executes safely
        wait_task(task, lambda: task.done, 600)
        # check that the task finished successfully (task.result) and did not have an error (task.error)
        if not task.result or task.error:
            if task.error:
                logger.error('plan execution failed with error: {} -> restarting'.format(task.error))
            elif not task.result:
                logger.error('plan execution failed non-specifically -> restarting')
            # restart the control server for good measure
            self._restart_control()
            return False

        return True


    # TODO: NEED TO UNDERSTAND THIS FUNCTION
    def _call(self, method, args, kwargs):
        # check that the control server is running
        if not self._check_control():
            return False

        # request a robot state update
        task = self.control.update()
        wait_task(task, lambda: task.done, 10)
        if task.error:
            logger.warn('robot state updated failed... planning from old state')

        # map to the plan name
        plan_method = 'plan' + method[0].upper() + method[1:]

        # invoke the planner
        task = getattr(self.planner, plan_method)(*args, **kwargs)
        target_plan_count = 3
        # the failed wait is only a problem if no plans were returned so ignore its return value
        wait_task(task, lambda: task.count_success() >= target_plan_count or task.all_done(), 600)
        if task.count_success() == 0:
            logger.warn('no plans available for {}'.format(method))
            return False
        logger.debug('got {}/{} plans'.format(task.count_success(), target_plan_count))

        # choose which plan to execute
        plans = [ job.result for job in task.get_success() ]
        # plans are tuples of (plan, goodness, limb)
        # limit to the first two tuple values since not all plans return an active limb
        best_plan = sorted(plans, key=lambda p: p[1])[0]
        logger.debug('best plan: {} ({})'.format(best_plan[0], best_plan[1]))

         # update the knowledge base with the active limb
        if len(best_plan) > 2:
            self.knowledge_base.active_limb = best_plan[2]
        else:
            self.knowledge_base.active_limb = None
        # update the knowledge base with the active grasp
        self.knowledge_base.active_grasp = best_plan[3]
        logger.debug('active limb changed to {}'.format(self.knowledge_base.active_limb))

        #save state in case you want to undo
        self.pre_execute_robot_state = copy.deepcopy(self.knowledge_base.robot_state)
        self.last_executed_path = best_plan[0]

        # execute the plan
        task = self.control.execute(best_plan[0])
        # check that the plan executes safely
        wait_task(task, lambda: task.done, 600)
        # check that the task finished successfully (task.result) and did not have an error (task.error)
        if not task.result or task.error:
            if task.error:
                logger.error('plan execution failed with error: {} -> restarting'.format(task.error))
            elif not task.result:
                logger.error('plan execution failed non-specifically -> restarting')
            # restart the control server for good measure
            self._restart_control()
            return False

        return True

class Master:
    def __init__(self, apc_order):
        self._create_knowledge_base()
        self._load_apc_order(apc_order)

        self.manager = MotionManager(self.knowledge_base, parallelism=1)
        self.manager.start()
        self.perception = PerceptionServer(self.knowledge_base, parallelism=1)

    def _create_knowledge_base(self):
        self.knowledge_base = KnowledgeBase()

        #import object dimension CSV file
        try:
            reader = csv.reader(open(os.path.join(knowledge_base_dir,'Object Dimensions.csv')))
            header = None
            for line in reader:
                if header == None:
                    header = line
                else:
                    self.knowledge_base.object_minimum_dims[line[0]] = float(line[1])*0.01
                    self.knowledge_base.object_maximum_dims[line[0]] = float(line[2])*0.01
                    self.knowledge_base.object_median_dims[line[0]] = float(line[3])*0.01
        except IOError:
            logger.info("Object dimensions unknown, try filling in Object Dimensions.csv")
            pass


        for k in [ 'bin_vantage_points', 'vantage_point_xforms', 'withdraw_point_xforms', 'bin_bounds' ]:
            path = os.path.join(knowledge_base_dir, '{}.json'.format(k))
            logger.info('loading {} from {}'.format(k, path))

            data = json.load(open(path))
            setattr(self.knowledge_base, k, data)

    def _load_apc_order(self, apc_order):
        global translation_to_berkeley
        # populate knowledge base
        self.knowledge_base.starting_bin_contents = apc_order['bin_contents'].copy()
        #do translation to berkeley
        for (b,objects) in self.knowledge_base.starting_bin_contents.iteritems():
            for i,o in enumerate(objects):
                if o in translation_to_berkeley:
                    objects[i] = translation_to_berkeley[o]
        self.knowledge_base.bin_contents = self.knowledge_base.starting_bin_contents.copy()

        self.order = apc_order['work_order'][:]
        for i,orderitem in enumerate(self.order):
            if orderitem['item'] in translation_to_berkeley:
                self.order[i]['item'] = translation_to_berkeley[orderitem['item']]

        # TODO: sort order by priority

    def _move_initial(self):
        # start at the initial pose so that sensors are free to localize
        logger.debug('returning to initial pose')
        if not self.manager.moveToInitialPose():
            logger.error('failed to move to initial pose')
            return False

        return True

    def _localize_shelf_and_order_bin(self):
        # find the shelf and order bin in parallel
        logger.debug('finding shelf and order bin xforms')
        shelf_task = self.perception.localizeShelf()
        order_bin_task = self.perception.localizeOrderBin()
        # this is accomplished by adding the localizeShelf and localizeOrderBin jobs into a single list
        task = JobManager(jobs=shelf_task.jobs + order_bin_task.jobs)

        # wait for both tasks to complete in parallel
        # ignore the wait result value since the outcome is determined by how many tasks were successful
        wait_task(task, lambda: (shelf_task.count_success() >= 1 and order_bin_task.count_success() >= 1) or task.all_done(), 15)
        if shelf_task.count_success() == 0 or order_bin_task.count_success() == 0:
            if shelf_task.count_success() == 0:
                logger.error('failed to localize shelf')
            if order_bin_task.count_success() == 0:
                logger.error('failed to localize order bin')
            return False

        # update the knowledge base with the shelf and order bin transforms
        self.knowledge_base.shelf_xform = shelf_task.get_success()[0].result
        self.knowledge_base.order_bin_xform = order_bin_task.get_success()[0].result
        logger.debug('found shelf at {}'.format(self.knowledge_base.shelf_xform))
        logger.debug('found order bin at {}'.format(self.knowledge_base.order_bin_xform))

        return True

    def _set_target(self, target_bin, target_object):
        # set the target in the knowledge base
        self.knowledge_base.target_bin = target_bin
        self.knowledge_base.target_object = target_object
        if target_bin and target_object:
            logger.info('attempting {} in {}'.format(target_object, target_bin))

    def _move_vantage_point(self, vantage_point):
        logger.debug('moving to {}'.format(vantage_point))
        if not self.manager.moveToVantagePoint(self.knowledge_base.target_bin, vantage_point):
            logger.error('failed to move to {}'.format(vantage_point))
            return False
        return True

    def _localize_object(self, vantage_point):
        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)

        # find the object from that vantage point
        task = self.perception.localizeSpecificObject(target_bin, target_object)
        # ignore the wait result value since the outcome is determined by how many tasks were successful
        wait_task(task, lambda: task.count_success() >= 1 or task.all_done(), 60)
        if task.count_success() == 0:
            logger.warn('could not find {} from {}'.format(target_object, vantage_point))
            return False

        # update knowledge_base with localized object transform, point cloud, and confusion matrix
        (object_xform, object_cloud, confusion_matrix) = task.get_success()[0].result
        self.knowledge_base.object_xforms[target_object] = object_xform
        self.knowledge_base.object_clouds[target_object] = object_cloud
        self.knowledge_base.last_confusion_matrix = confusion_matrix
        logger.info('found {} at {}'.format(target_object, object_xform))
        logger.debug('found {} with {} points'.format(target_object, len(object_cloud)))

        return True

    def _find_object(self):
        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)

        # search for the object from all available vantage points
        object_found = False
        # vantage points are ordered by goodness
        for vantage_point in self.knowledge_base.bin_vantage_points[target_bin]:
            if not self._move_vantage_point(vantage_point):
                return False

            if self._localize_object(vantage_point):
                object_found = True
                break
            else:
                pass
                # try another vantage point

        if not object_found:
            logger.error('failed to find {} from any vantage point'.format(target_object))
            return False

        return True

    def _grasp_object(self):
        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)

        # grasp the object
        logger.debug('grasping {} in {}'.format(target_object, target_bin))
        if not self.manager.graspObjectInBin(target_bin, target_object):
            logger.error('failed to grasp {} in {}'.format(target_object, target_bin))
            return False
        # update knowledge base to reflect grasped object
        self.knowledge_base.grasped_object = target_object
        logger.info('grasped {}!'.format(target_object))

        return True

    def _retrieve_object(self):
        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)

        # move the grasped object to the order bin
        logger.debug('moving {} to order bin'.format(target_object))
        if not self.manager.moveObjectToOrderBin(self.knowledge_base.active_limb):
            logger.error('failed to move {} to order bin'.format(target_object))
            return False
        # update the knowledge base with the object removal
        self.knowledge_base.bin_contents[target_bin].remove(target_object)
        del self.knowledge_base.object_xforms[target_object]
        del self.knowledge_base.object_clouds[target_object]
        self.knowledge_base.grasped_object = None

        return True

    def _undo_failed_retrieve(self):
        logger.debug('undoing prior grasp due to failed retrieve')
        self.manager.undo_last_motion()
        self.knowledge_base.grasped_object = None

    def run(self, target_bin, target_object):
        if not self._move_initial() or not self._localize_shelf_and_order_bin():
            return False

        self._set_target(target_bin, target_object)

        if not self._find_object() or not self._grasp_object():
            return False

        if not self._retrieve_object():
            self._undo_failed_retrieve()
            return False

        self._set_target(None, None)

        # this order item is completed
        logger.info('completed order for {}'.format(target_object))

        return True
