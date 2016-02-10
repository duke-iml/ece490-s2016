import sys; sys.path.append('.')
from Motion import motion

#manual motion setup
motion.setup(mode='kinematic',klampt_model='klampt_models/baxter_with_parallel_gripper_col.rob',libpath='./')
#automatic motion setup
#from Motion import config
#config.setup(parse_sys=False)

motion.robot.startup()
try:
    while True:
        raw_input("Press enter to print sensed left arm positions > ")
        print motion.robot.left_limb.sensedPosition()
except KeyboardInterrupt:
    print "Shutting down..."



