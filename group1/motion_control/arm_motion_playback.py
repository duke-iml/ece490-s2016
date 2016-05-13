#!/usr/bin/python
import sys; sys.path.append('.')
import time
from common.Motion import motion
import config
from klampt.trajectory import Trajectory

fn = "arm_motion.traj"
speed = 100
trim = 0

print "Plays back an arm motion"
print "Usage: arm_motion_playback.py FILE [SPEED] [TRIM]"
if len(sys.argv) <= 1:
    print "Please specify a motion file"
    exit(0)
if len(sys.argv) > 2:
    speed = float(sys.argv[2])
if len(sys.argv) > 3:
    trim = float(sys.argv[3])

fn = sys.argv[1]
print "Loading motion from",fn
traj = Trajectory()
traj.load(fn)

if len(traj.milestones[0]) != 14:
    print "Error loading arms trajectory, size is not 14"

#simple motion setup
config.setup(parse_sys=False)

motion.robot.startup()
try:
    if trim == 0:
        print "Moving to start, 20%% speed..."
    else:
        print "Moving to config at time %f, 20%% speed..."%(trim,)
    motion.robot.arms_mq.setRamp(traj.eval(trim),speed=0.2)
    while motion.robot.arms_mq.moving():
        time.sleep(0.1)
    print "Starting motion..."
    """
    #this is direct interpolation in Python at 50 Hz
    t0 = time.time()
    while time.time()-t0 < traj.times[-1]/speed:
        q = traj.eval((time.time()-t0)/speed)
        motion.robot.left_arm.setPosition(q[0:7])
        motion.robot.right_arm.setPosition(q[7:14])
        time.sleep(0.02)
    """
    #this uses the motion queue
    t = trim
    q = traj.milestones[0]
    for i in range(len(traj.times)):
        if traj.times[i] > t:
            motion.robot.arms_mq.appendLinear((traj.times[i]-t)/speed,traj.milestones[i])
            t = traj.times[i]
            q = traj.milestones[i]
    while motion.robot.arms_mq.moving():
        time.sleep(0.1)        
except KeyboardInterrupt:
    print "Exiting."
