from Motion import motion

motion.setup(mode='client',klampt_model='klampt_models/baxter_with_parallel_grippers',libpath='.')

motion.robot.startup()
try:
    while True:
        raw_input()
        print motion.robot.left_limb.sensedPosition()
except KeyboardInterrupt:
    motion.robot.stopMotion()


