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
        print "Enabled?",robot.left_gripper.enabled()
        robot.left_gripper.close()
    else:
        print "Enabled?",robot.right_gripper.enabled()
        robot.right_gripper.close()
    time.sleep(1.0)
    exit(0)

