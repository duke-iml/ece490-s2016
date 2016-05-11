import rospy
import sys
from reflex_sf_msgs.msg import Command
 
if len(sys.argv) <= 1:
    print "Usage: python dof_tour.py HAND_NAME"
    print "Where HAND_NAME is either hand1 or hand2"
    exit(0)

hand = sys.argv[1]

rospy.init_node('reflex_sf_dof_tour')
pub = rospy.Publisher('/reflex_%s/command'%(hand,), Command, queue_size=10)

FINGER_CLOSED = 4.6
FINGER_PINCH = 3.5 

PRESHAPE_CYLINDER = 0
PRESHAPE_SPHERICAL = 1.5
PRESHAPE_PINCH = 2.5

opencmd = Command()
opencmd.position = [0,0,0,0]
opencmd.velocity = [1,1,1,1]
opencmd.force = [1,1,1,1]

cmd = Command()
cmd.position = [0,0,0,0]

# preshape
for i in range(100):
    if rospy.is_shutdown():
        exit(0)
    print "Opening..."
    pub.publish(opencmd)
    rospy.sleep(2)
    cmd.position = [FINGER_CLOSED,FINGER_CLOSED,FINGER_CLOSED,0]
    cmd.force = [0.5,0.5,0.5,1.0]
    cmd.velocity = [0.1,0.1,0.1,0.1]
    print "Closing..."
    pub.publish(cmd)
    rospy.sleep(5)
    #cmd.position = [0,0,0,PRESHAPE_SPHERICAL]
    #pub.publish(cmd)
    #rospy.sleep(2)
    #cmd.position = [0,0,0,PRESHAPE_PINCH]
    #pub.publish(cmd)
    #rospy.sleep(2)
exit(0)

# finger 1 (right-hand index) close
rospy.sleep(5)
pub.publish(opencmd)
rospy.sleep(1)
cmd.position = [FINGER_CLOSED,0,0,0]
pub.publish(cmd)
rospy.sleep(1)
pub.publish(opencmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# finger 2 (right-hand middle) close
cmd.position = [0,FINGER_CLOSED,0,0]
pub.publish(cmd)
rospy.sleep(1)
pub.publish(opencmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# finger 3 (thumb) close
cmd.position = [0,0,FINGER_CLOSED,0]
pub.publish(cmd)
rospy.sleep(1)
pub.publish(opencmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# preshape
cmd.position = [0,0,0,PRESHAPE_SPHERICAL]
pub.publish(cmd)
rospy.sleep(1)
cmd.position = [0,0,0,PRESHAPE_PINCH]
pub.publish(cmd)
rospy.sleep(1)
pub.publish(opencmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# hand closed in cylindrical power grasp
cmd.position = [FINGER_CLOSED,FINGER_CLOSED,FINGER_CLOSED,0]
pub.publish(cmd)
rospy.sleep(1)
# hand open
pub.publish(opencmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# preshape hand for pinch
cmd.position = [0, 0, 0, PRESHAPE_PINCH]
pub.publish(cmd)
rospy.sleep(1)
# pinch grasp
cmd.position = [FINGER_PINCH, FINGER_PINCH, 0, PRESHAPE_PINCH]
pub.publish(cmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# hand open (pinch grasp)
cmd.position = [0, 0, 0, PRESHAPE_PINCH]
pub.publish(cmd)
rospy.sleep(1)
if rospy.is_shutdown():
    exit(0)
# hand open (cylindrical grasp)
pub.publish(opencmd)
rospy.sleep(1)
