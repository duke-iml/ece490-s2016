import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import json
import integration
import planning
from integration.master import Master
from planning.interface import scoopOrGrasp
from collections import defaultdict
import time
import logging
import math
import csv
from integration.io import pcd
from klampt import se3

logger = logging.getLogger(__name__)

#total time for run
total_time = 60.0*20.0

sensing_order = 'ALDIGFJCKBHE'

#estimated time to scoop / item
expected_scoop_time_per_item = 50
#estimated time to grasp / item
expected_grasp_time_per_item = 50

objects = ["oreo_mega_stuf",
                    "champion_copper_plus_spark_plug",
                    "expo_dry_erase_board_eraser",
                    "kong_duck_dog_toy",
                    "genuine_joe_plastic_stir_sticks",
                    "munchkin_white_hot_duck_bath_toy",
                    "crayola_64_ct",
                    "mommys_helper_outlet_plugs",
                    "sharpie_accent_tank_style_highlighters",
                    "kong_air_dog_squeakair_tennis_ball",
                    "stanley_66_052",
                    "safety_works_safety_glasses",
                    "dr_browns_bottle_brush",
                    "laugh_out_loud_joke_book",
                    "cheezit_big_original",
                    "paper_mate_12_count_mirado_black_warrior",
                    "feline_greenies_dental_treats",
                    "elmers_washable_no_run_school_glue",
                    "mead_index_cards",
                    "rollodex_mesh_collection_jumbo_pencil_cup",
                    "first_years_take_and_toss_straw_cups",
                    "highland_6539_self_stick_notes",
                    "mark_twain_huckleberry_finn",
                    "kygen_squeakin_eggs_plush_puppies",
                    "kong_sitting_frog_dog_toy"]

#APC rewards
rewards = {"place_1_item":10,
           "place_2_item":15,
           "place_multi_item":20,
           "drop":-3,
           "place_wrong_item":-12}

#APC bonus rewards
bonus_items = {
    'mark_twain_huckeberry_finn' :		3,
    'kygen_squeakin_eggs_plush_puppies':		1,
    'kong_air_dog_squeakair_tennis_ball':		1,
    'dr_browns_bottle_brush':		2,
    'kong_duck_dog_toy':		1,
    'kong_sitting_frog_dog_toy':		1,
    'laugh_out_loud_joke_book':		1,
    'munchkin_white_hot_duck_bath_toy':		1,
    'safety_works_safety_glasses':		1,
    'stanley_66_052':		3,
    'rollodex_mesh_collection_jumbo_pencil_cup':		2
    }

#Estimated probability of dropping objects
p_drop = {"scoop":dict((o,0.1) for o in objects),
          "pinch":dict((o,0.2) for o in objects),
          "power":dict((o,0.1) for o in objects),
          "tilt":dict((o,0.1) for o in objects)}
p_drop_miss = 0.2

p_scoop_success = {}
p_pinch_success = {}
p_power_success = {}

p_pickup_success = {'scoop':p_scoop_success,
                        'pinch':p_pinch_success,
                        'power':p_power_success,
                        'tilt':p_power_success}

def load_strategy_constants(fn = "Strategy Optimization.csv"):
    try:
        reader = csv.reader(open(fn,'r'))
    except IOError:
        raise
    header = None
    powercol = None
    pinchcol = None
    scoopcol = None
    perceivecol = None
    for line in reader:
        if header==None:
            header = line
            assert 'Item' == header[0]
            assert 'Scoop' in header
            assert 'Power Grasp' in header
            assert 'Pinch Grasp' in header
            assert 'Perceive' in header
            scoopcol = header.index('Scoop')
            powercol = header.index('Power Grasp')
            pinchcol = header.index('Pinch Grasp')
            perceivecol = header.index('Perceive')
        else:
            assert len(line)>=5
            assert line[0] in objects,'Object "'+line[0]+'" appears invalid'
            obj = line[0]
            p_scoop_success[obj] = float(line[scoopcol])
            #p_detect_fail[obj] = 1.0 - float(line[perceivecol])
            p_pinch_success[obj] = float(line[pinchcol])
            p_power_success[obj] = float(line[powercol])

class PlanningMaster(Master):
    def __init__(self, apc_order, test=False):
        if not test:
            Master.__init__(self,apc_order)
        else:
            self._create_knowledge_base()
            self._load_apc_order(apc_order)
        load_strategy_constants("apc/high_level_planning/Strategy Optimization.csv")
        self.order_by_bin = defaultdict(list)
        for orderitem in self.order:
            self.order_by_bin[orderitem['bin']].append(orderitem['item'])

        self.cache = False
        print "Loading prior point clouds"
        if self.cache:
            for bin,objects in self.knowledge_base.bin_contents.iteritems():
                for o in objects:
                    pcd_file_name = '/tmp/'+o+'_'+bin+".pcd"
                    if os.path.exists(pcd_file_name):
                        self.knowledge_base.object_xforms[o] = se3.identity()
                        self.knowledge_base.object_clouds[o] = pcd.read(open(pcd_file_name))[1]
                        #self.manager.control.update()
                        #time.sleep(0.5)

    def run_all(self):
        global logger
        global total_time,rewards,bonus_items,p_drop,p_drop_miss,p_pickup_success
        t0 = time.time()

        for orderitem in self.order:
            print "  ",orderitem["bin"],orderitem["item"]
        print len(self.order),"Items to get"
        logger.info("Running localization, time 0")

        if not self._move_initial() or not self._localize_shelf_and_order_bin():
            return False

        #first step: run all the localizations
        logger.info("Running localization, time 0")
        not_found_objects = []
        confusion_matrices = {}
        global sensing_order
        for bin in ['bin_'+x for x in sensing_order]:
            # if bin[4] in 'CFIL':
            #     continue
            logger.info("bin {}".format(bin))
            if len(self.order_by_bin[bin])==0:
                continue
                print "No order item in bin",bin
                self._set_target(bin,None)
                if not self._move_vantage_point(bin+"_center"):
                    logger.info("Couldn't move to vantage point %s... pressing on."%(bin,))
            else:
                for i,o in enumerate(self.order_by_bin[bin]):
                    print bin,o
                    confusion_matrix_file = "/tmp/confusion_matrices_"+o+"_"+bin+".json"
                    if o in self.knowledge_base.object_clouds:
                        if self.cache:
                            try:
                                f = open("/tmp/confusion_matrices_"+o+"_"+bin+".json",'r')
                                confusion_matrices[(bin,o)] = json.load(f)
                                logger.info("Skipping localization for object "+o)
                                f.close()
                                continue
                            except IOError:
                                pass
                            del self.knowledge_base.object_xforms[o]
                            del self.knowledge_base.object_clouds[o]
                    #confusion_matrices[(bin,o)] = dict((o2,1.0/len(self.knowledge_base.bin_contents[bin])) for o2 in self.knowledge_base.bin_contents[bin])
                    #continue
                    self._set_target(bin,o)
                    if i==0:
                        #if not self._move_vantage_point(bin+"_center"):
                        self._set_target(bin,o)
                        if not self._find_object():
                            logger.info("Couldn't find object %s in bin %s... pressing on."%(o,bin))
                            not_found_objects.append((bin,o))
                            if self.cache:
                                try:
                                    os.remove(confusion_matrix_file)
                                except IOError:
                                    pass
                            continue
                    else:
                        if not self._localize_object(bin+"_center"):
                            logger.info("Couldn't localize object %s in bin %s"%(o,bin))
                            not_found_objects.append((bin,o))
                            if self.cache:
                                try:
                                    os.remove(confusion_matrix_file)
                                except IOError:
                                    pass
                            continue
                    print "Confusion matrix",self.knowledge_base.last_confusion_matrix
                    confusion_matrices[(bin,o)] = self.knowledge_base.last_confusion_matrix
                    if self.cache:
                        #save confusion matrix to disk
                        f = open(confusion_matrix_file,'w')
                        f.write(json.dumps(self.knowledge_base.last_confusion_matrix))
                        f.write("\n")
                        f.close()
        t = time.time()-t0
        logger.info("Localization done, time %f. %f time remaining"%(t,total_time-t))

        #second step: compute the optimal order
        best_operations = dict()
        expected_reward = dict()
        skipping = []
        for orderitem in self.order:
            b,o = orderitem['bin'],orderitem['item']
            if (b,o) not in confusion_matrices:
                skipping.append((b,o))
                continue
            best_op = scoopOrGrasp(o,b,self.knowledge_base)
            if best_op == None:
                logger.info("scoopOrGrasp reported no valid manipulation for object %s in bin %s",o,b)
                continue
            best_operations[(b,o)] = best_op
            epoints = 0.0
            confusion = confusion_matrices[(b,o)]
            num_items_in_bin = len(self.knowledge_base.bin_contents[b])
            success_points = 0.0
            if num_items_in_bin==1:
                success_points = rewards["place_1_item"]
            elif num_items_in_bin==2:
                success_points = rewards["place_2_item"]
            elif num_items_in_bin >= 3:
                success_points = rewards["place_multi_item"]
            else:
                raise ValueError("empty bin?")
            
            if o in bonus_items:
                success_points += bonus_items[o]
            p_drop_o = p_drop[best_op][o]*1.0
            p_drop_o_in_bin = p_drop[best_op][o]*(1.0-p_drop_miss)
            p_drop_o_miss = p_drop[best_op][o]*p_drop_miss
            p_successful_place = (1.0-p_drop_o)*p_pickup_success[best_op][o]
            p_successful_place_dropped = p_drop_o_in_bin*p_pickup_success[best_op][o]
            
            for (other,prob) in confusion.iteritems():
                if other==o:
                    #probability of success
                    epoints += prob*(success_points*p_successful_place+p_drop_o_in_bin*(success_points-rewards["drop"]))
                else:
                    epoints += rewards["place_wrong_item"]*prob*p_pickup_success[best_op][other]
            
            expected_reward[(b,o)] = epoints
        #print expected_reward.keys()
        #print expected_reward.values()
        #print skipping
        #assert len(expected_reward) + len(skipping) == len(self.order)
        sorted_rewards = list(reversed(sorted([(v,k) for (k,v) in expected_reward.iteritems()])))
        scoop_order = [k for (v,k) in sorted_rewards if best_operations[k]=='scoop' and v > 0]
        grasp_order = [k for (v,k) in sorted_rewards if best_operations[k] in ['power','pinch','tilt'] and v > 0]
        skipping += [k for (v,k) in sorted_rewards if v <= 0]
        #assert len(self.order) == len(scoop_order) + len(grasp_order) + len(skipping)
        logger.info("Scoop proceeding in the following order:")
        for (b,o) in scoop_order:
            logger.info("  Object %s in bin %s reward %f"%(b,o,expected_reward[(b,o)]))
        logger.info("Grasp proceeding in the following order:")
        for (b,o) in grasp_order:
            logger.info("  Object %s in bin %s reward %f"%(b,o,expected_reward[(b,o)]))

        logger.info("Skipping the following objects:")
        for (b,o) in skipping:
            if (b,o) in expected_reward:
                logger.info("  Object %s in bin %s reward %f"%(b,o,expected_reward[(b,o)]))
            else:
                logger.info("  Object %s in bin %s reward %f"%(b,o,0.0))
    
        self._move_initial()
        self.knowledge_base.active_limb = 'left'
        #start grasping
        for (b,o) in grasp_order:
            #decide whether to switch to scooping
            t = time.time()-t0
            t_left = total_time - t
            s = self.optimal_stopping(t_left,[expected_reward[x] for x in grasp_order[i:]],[expected_reward[x] for x in scoop_order])
            if s == 0:
                logger.info("Deciding to switch to scooping")
                break

            self._set_target(b,o)
            #TODO: do we want to re-find the object?
            #self._find_object()
            if self._grasp_object():
                if self._retrieve_object():
                    # this order item is completed 
                    logger.info('completed order for {}'.format(o))
                else:
                    logger.info("Failed to retrieve object")
                    self._undo_failed_retrieve()
            else:
                logger.info("Failed to plan grasp for object")
            # this order item is completed
            logger.info('completed order for {}'.format(o))
            self._set_target(None, None)

        #start scooping
        if len(scoop_order) != 0 and False:
            #pick up tray
            self.knowledge_base.active_limb = 'right'
            self.manager.PickUpOrderTray(True)
            for i,(b,o) in enumerate(scoop_order):
                #test whether to switch to grasping
                #t = time.time()-t0
                #t_left = total_time - t
                #s = self.optimal_stopping(t_left,[expected_reward[x] for x in scoop_order[i:]],[expected_reward[x] for x in grasp_order])
                #if s == 0:
                #    logger.info("Deciding to switch to grasping")
                #    break

                self._set_target(b,o)
                #TODO: do we want to re-sign the object?
                #self._find_object()
                if self._grasp_object():
                    if self._retrieve_object():
                        # this order item is completed 
                        logger.info('completed order for {}'.format(o))
                    else:
                        logger.info("Failed to retrieve object")
                        self._undo_failed_retrieve()
                else:
                    logger.info("Failed to plan scoop for object")
                self._set_target(None, None)
            #put down tray
            self.manager.PickUpOrderTray(False)
 

        logger.info("Done.")
        #TODO: go back and re-sense

    def optimal_stopping(self,t_left,scoop_reward,grasp_reward):
        global expected_scoop_time_per_item,expected_grasp_time_per_item
        max_items_to_scoop = min(len(scoop_reward),int(math.floor(t_left/expected_scoop_time_per_item)))
        max_items_to_grasp = min(len(grasp_reward),int(math.floor(t_left/expected_grasp_time_per_item)))
        if max_items_to_scoop == 0:
            return 0
        rewards = []
        #dynamic programming
        rscoop = 0.0
        rgrasp = sum(grasp_reward[:max_items_to_grasp])
        tconsumed_scoop = 0.0
        for i in range(max_items_to_scoop):
            rewards.append(rscoop+rgrasp)
            tconsumed_scoop += expected_scoop_time_per_item
            
            num_items_to_grasp = min(len(grasp_reward),int(math.floor(t_left - tconsumed_scoop)/expected_grasp_time_per_item))
            rgrasp -= sum(grasp_reward[num_items_to_grasp:max_items_to_grasp])
            
            max_items_to_grasp = num_items_to_grasp
            rscoop += scoop_reward[i]
        print rewards
        return max([(r,i) for (i,r) in enumerate(rewards)])[1]


if __name__=="__main__":
    import logging, traceback, sys
    from rainbow_logging_handler import RainbowLoggingHandler

    #logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')
    # handler = RainbowLoggingHandler(sys.stderr)
    # handler.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)s: %(message)s'))
    # logger = logging.getLogger()
    # logger.setLevel(logging.DEBUG)
    # logger.addHandler(handler)

    # log everything to the file
    import time, os
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(filename)-15s:%(lineno)-4d %(levelname)-8s %(message)s',
        datefmt='%m-%d-%Y %H:%M:%S',
        filename='/tmp/hlp-{}-{}.log'.format(int(time.time()), os.getpid()),
        filemode='w'
    )

    # log specific things to the console
    console = RainbowLoggingHandler(sys.stderr)
    console.setLevel(logging.DEBUG)
    console.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)-8s: %(message)s'))
    logging.getLogger('planning.interface').addHandler(console)
    logging.getLogger('integration.master').addHandler(console)
    logging.getLogger('__main__').addHandler(console)
    
    # log all warnings or higher
    consoleError = RainbowLoggingHandler(sys.stderr)
    consoleError.setLevel(logging.WARN)
    consoleError.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)-8s: %(message)s'))
    logging.getLogger().addHandler(consoleError)
    
    logging.getLogger('OpenGL').setLevel(99)
    # logging.getLogger('integration.jobs').setLevel(logging.WARNING)
    # logging.getLogger('integration.visualization').setLevel(logging.WARNING)
    # logging.getLogger('integration.interface').setLevel(logging.WARNING)
    # logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
    # logging.getLogger('integration.io.pcd').setLevel(logging.WARNING)
    # logging.getLogger('integration.camera.client').setLevel(logging.WARNING)
    # logging.getLogger('integration.camera.packet').setLevel(logging.WARNING)
    # logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
    # logging.getLogger('integration.control_server').setLevel(logging.ERROR)
    # logging.getLogger('integration.master').setLevel(logging.INFO)
    # logging.getLogger('perception.segmentation.shelf').setLevel(logging.WARNING)
    # logging.getLogger('perception.segmentation.icp').setLevel(logging.WARNING)
    # logging.getLogger('perception.segmentation.blob').setLevel(logging.WARNING)
    # logging.getLogger('perception.interface').setLevel(logging.WARNING)
    # logging.getLogger('planning.control').setLevel(logging.WARNING)

    # speed up the tests
    from integration.interface import fake
    fake.delay_scale = 0
    fake.fail_scale = 100
    
    fn = "launch_files/apc.json"
    if len(sys.argv) > 1:
        fn = sys.argv[1]
    f = open(fn,'r')
    apc_order = json.load(f)
    #master = PlanningMaster(apc_order,True)
    master = PlanningMaster(apc_order,False)
    master.run_all()
    #print master.optimal_stopping(6*60.0,[5,5,5,5,4,4,4,3,2,2,2,2,2,2,2,1,1,1,1,1,1],[7,6,6,6,4,4,4,3,3,3,3,3,2,2,2,2,2,1,1,1])
