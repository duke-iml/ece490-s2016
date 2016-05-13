from Motion import motion
import time

print "Testing APC Motion..."
robot = motion.setup(libpath="./",klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob")
print "Starting up..."
res = robot.startup()
print "Start up result:",res
if res:
    print "Is robot started?",robot.isStarted()
    print
    """
    while True:
        raw_input('Press enter to get positions')
        print "Left arm:",robot.left_limb.sensedPosition()
        print "Right arm:",robot.right_limb.sensedPosition()
    """
    for i in range(20):
        robot.left_gripper.command([1,1,1,1],[1,1,1,1],[1,1,1,1])
        time.sleep(1.0)
        robot.left_gripper.command([0,0,0,1],[1,1,1,1],[1,1,1,1])
        time.sleep(1.0)
    robot.left_gripper.command([1,1,1,1],[1,1,1,1],[1,1,1,1])
    exit(0)
 
    index = 5
    qref = robot.right_limb.sensedPosition()
    while True:
        """
        for i in range(50): 
            robot.right_limb.positionCommand([0,0,0,0,0,float(i)/50*2,0])
            time.sleep(0.1)
        for i in range(50):
            robot.right_limb.positionCommand([0,0,0,0,0,2-float(i)/50*2,0])
            time.sleep(0.1)
        """
        print "Moving down..."
        qref[index] = 1
        robot.right_mq.setRamp(qref)
        while robot.right_mq.moving():
            time.sleep(0.1)
        time.sleep(1)
        print "Moving up..."
        qref[index] = 0
        robot.right_mq.setRamp(qref)
        while robot.right_mq.moving():
            time.sleep(0.1)
        time.sleep(1)

    robot.shutdown()
