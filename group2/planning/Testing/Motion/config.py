#################### SYSTEM CONFIGURATION VARIABLES #######################

#change this to "client" to use the MotionServer_X program
#change this to "kinematic" to test in kinematic mode
#change this to "physical" if you are ready to test on the real robot
#mode = 'client'
#mode = 'kinematic'
mode = 'physical'

#relative path, assumes everything is run from the common directory
klampt_model = "../klampt_models/baxter_with_new_spatula_col2.rob"
# klampt_model = "../klampt_models/baxter_with_parallel_gripper_col.rob"
#klampt_model = "klampt_models/hstar_amp1_col.rob"

#Motion server address.  Use this if you are using a server on another
#machine.
motion_server_addr = None

def parse_args():
    import sys
    global mode,klampt_model
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if len(sys.argv) > 2:
            klampt_model = sys.argv[2]

def setup(parse_sys=True):
    global mode,klampt_model,motion_server_addr
    import os
    import motion
    motion_root = os.getenv("MOTION_API_PATH","./")
    if parse_sys:
        parse_args()
    print "Loading Motion Module model",klampt_model
    motion.setup(mode=mode,klampt_model=klampt_model,libpath=motion_root,server_addr=motion_server_addr)
