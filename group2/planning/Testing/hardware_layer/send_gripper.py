from Motion import motion
import sys
import time

print "Testing APC Motion..."
robot = motion.setup(libpath="./",klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob")
print "Starting up..."
res = robot.startup()
print "Start up result:",res
if res:
    print "Is robot started?",robot.isStarted()
    print
    gripper = sys.argv[1]
    amount = float(sys.argv[2])
    pinch = 0
    if len(sys.argv) >= 4:
        if sys.argv[3] == 'pinch':
            pinch = 1
    if gripper == 'left':
        print "Enabled?",robot.left_gripper.enabled()
        if pinch:
            robot.left_gripper.command([amount,amount,.2,0],[1]*4,[1]*4)
        else:
            robot.left_gripper.command([amount,amount,amount,1],[1]*4,[1]*4)
    else:
        print "Enabled?",robot.right_gripper.enabled()
        if pinch:
            robot.right_gripper.command([amount,amount,.2,0],[1]*4,[1]*4)
        else:
            robot.right_gripper.command([amount,amount,amount,1],[1]*4,[1]*4)
    time.sleep(1.0)
    exit(0)

