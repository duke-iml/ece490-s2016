import sys; sys.path.append('.')
import time
from Motion import motion
from Motion import config

#save at 10Hz, by default.  Set to 0 to save by pressing enter
rate = 10.0
#save here, by default
fn = "arm_motion.traj"

print "Saves arm motions to disk for later inspection and playback"
print "Usage: arm_motion_record.py [RATE] [FN]"
print " [RATE] can be 0 to only save when enter is pressed"
print " [FN] is the file to save to"
if len(sys.argv) > 1:
    rate = float(sys.argv[1])

if rate > 0:
    print "Saving motion at %g Hz... "%(rate,)
else:
    print "Saving motion when enter is pressed"

if len(sys.argv) > 2:
    fn = sys.argv[2]

print "Saving motion to",fn
f = open(fn,'w')

#simple motion setup
config.setup(parse_sys=False)

motion.robot.startup()
lastt = None
lastq = None
try:
    print "Beginning capture loop, press Ctrl+C to quit"
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t < 1e-3: t = 0
        q = motion.robot.left_limb.sensedPosition()+motion.robot.right_limb.sensedPosition()
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
