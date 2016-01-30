from Motion import motion
import sys

print "Testing APC Motion..."
robot = motion.setup(libpath="./",klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob")
print "Starting up..."
res = robot.startup()
print "Start up result:",res
if res:
    print "Is robot started?",robot.isStarted()
    print
    gripper = sys.argv[1]
    if gripper == 'left':
        robot.left_gripper.open()
    else:
        robot.right_gripper.open()
    time.sleep(1.0)
    exit(0)

