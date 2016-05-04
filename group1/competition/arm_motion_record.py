#!/usr/bin/python
import sys; sys.path.append('.')
import time
from common.Motion import motion
import config

#save at 10Hz, by default.  Set to 0 to save by pressing enter
rate = 20.0
#save here, by default
fn = "arm_motion.traj"
#commanded or sensed?
switch = 's'

print "Saves arm motions to disk for later inspection and playback"
print "Usage: arm_motion_record.py TYPE [RATE] [FN]"
print " TYPE can be 'sensed' or 'commanded', or 's' or 'c' for short"
print " [RATE] can be 0 to only save when enter is pressed"
print " [FN] is the file to save to"

if len(sys.argv) > 1:
    switch = sys.argv[1]
if switch == 's':
    switch = 'sensed'
if switch == 'c':
    switch = 'commanded'
if switch not in ['sensed','commanded']:
    print "TYPE argument must be either 'sensed' or 'commanded'"
    exit(1)

if len(sys.argv) > 2:
    rate = float(sys.argv[2])

if len(sys.argv) > 3:
    fn = sys.argv[3]


#simple motion setup
config.setup(parse_sys=False)
motion.robot.startup()

if rate > 0:
    print "Saving %s motion at %g Hz to %s "%(switch,rate,fn)
else:
    print "Saving %s motion when enter is pressed to %s"%(switch,fn)

f = open(fn,'w')

lastt = None
lastq = None
try:
    print "Beginning capture loop, press Ctrl+C to quit"
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t < 1e-3: t = 0
        if switch == 'sensed':
            q = motion.robot.left_limb.sensedPosition()+motion.robot.right_limb.sensedPosition()
        else:
            q = motion.robot.left_limb.commandedPosition()+motion.robot.right_limb.commandedPosition()
        if q != lastq:
            if lastq is not None:
                f.write(str(lastt) + '\t' + str(len(lastq)) + '\t'+ ' '.join(str(v) for v in lastq)+'\n')
            f.write(str(t) + '\t' + str(len(q)) + '\t'+ ' '.join(str(v) for v in q)+'\n')
            lastt = t
            lastq = q
        if rate <= 0:
            raw_input("Press enter to capture > ")
        else:
            time.sleep(1.0/rate)
except KeyboardInterrupt:
    f.close()
    print "Saved %g seconds of motion. Exiting."%(time.time()-t0,)
    pass
