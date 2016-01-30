import sys
from Motion import motion
from klampt import *
from klampt import loader
import time

skip_gaps = True
max_gap = 2.0

def read_commands(fn):
    f = open(fn,'r')
    if not f:
        print "Could not open motion file",fn
        raise RuntimeError()

    commands = []
    for line in f.readlines():
        if len(line.strip())==0: continue  #empty line
        items = line.strip().split()
        if len(items) < 2:
            raise RuntimeError("Line does not have at least time, name in tab-separated columns")
        t = float(items[0])
        function = items[1]
        rawargs = items[2:]
        args = []
        if function == 'setMilestone' or function == 'appendMilestone' or function == 'PID' or function == 'startConfig':
            len1 = int(rawargs[0])
            if 1+len1 > len(rawargs):
                raise RuntimeError("Invalid length "+str(len1))
            args.append([float(v) for v in rawargs[1:1+len1]])
            if 2+len1 < len(rawargs):
                len2 = int(rawargs[2+len1])
                if 3+len1+len2 > len(rawargs):
                    raise RuntimeError("Invalid length "+str(len1))
                args.append([float(v) for v in rawargs[3+len1:3+len1+len2]])
        elif function == 'commandGripper':
            args.append(rawargs[0])
            length = int(rawargs[1])
            args.append([float(v) for v in rawargs[2:2+length]])
        else:
            args = rawargs
        commands.append({'time':t,'function':function,'args':args})
    f.close()
    print "Read",len(commands),"commands from file",fn
    return commands

def timeshift_commands(commands,offset=0,scale=1):
    for c in commands:
        c['time'] = offset + c['time']*scale

klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob"
world = WorldModel()
world.readFile(klampt_model)
robot = world.robot(0)
left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
left_arm_link_indices = [robot.getLink(l).index for l in left_arm_link_names]
right_arm_link_indices = [robot.getLink(l).index for l in right_arm_link_names]

def send_command(command,robot):
    function = command['function']
    args = command['args']
    if function == 'startConfig':
        #move slowly to start config
        q = args[0]
        robot.left_mq.setRamp([q[i] for i in left_arm_link_indices])
        robot.right_mq.setRamp([q[i] for i in right_arm_link_indices])
        while robot.left_mq.moving() or robot.right_mq.moving():
            print "Waiting to move to startConfig..."
            time.sleep(1.0)
    elif function == 'PID':
        q = args[0]
        v = args[1]
        #can only do position
        robot.left_limb.positionCommand([q[i] for i in left_arm_link_indices])
        robot.right_limb.positionCommand([q[i] for i in right_arm_link_indices])
    elif function == 'setMilestone':
        q = args[0]
        if len(args)>=2:
            v = args[1]
        else:
            v = None
        #can only do position
        print "Sending setRamp command"
        print "    ",[q[i] for i in left_arm_link_indices]
        print "    ",[q[i] for i in right_arm_link_indices]
        robot.left_mq.setRamp([q[i] for i in left_arm_link_indices],0.65)
        robot.right_mq.setRamp([q[i] for i in right_arm_link_indices],0.65)
    elif function == 'appendMilestone':
        q = args[0]
        if len(args)>=2:
            v = args[1]
        else:
            v = None
        #can only do position
        print "Sending appendRamp command"
        print "    ",[q[i] for i in left_arm_link_indices]
        print "    ",[q[i] for i in right_arm_link_indices]
        robot.left_mq.appendRamp([q[i] for i in left_arm_link_indices],0.65)
        robot.right_mq.appendRamp([q[i] for i in right_arm_link_indices],0.65)
    elif function == 'commandGripper':
        limb,p = args
        #hack because the right hand is on USB0
        limb = 'right'
        if len(p)==1 and robot.right_gripper.numDofs()==4:
            #don't close all the way
            u = p[0]*0.8+0.2
            p = [u,u,u,1]
        print "Sending gripper command",limb,p
        if limb == 'left':
            robot.left_gripper.command(p,[1]*len(p),[1]*len(p))
        elif limb == 'right':
            robot.right_gripper.command(p,[1]*len(p),[1]*len(p))
        else:
            raise RuntimeError("Invalid limb "+limb)
    else:
        raise RuntimeError("Invalid function "+function)

fn = sys.argv[1]
commands = read_commands(fn)

assert commands[0]['function'] == 'startConfig', "Commands must start from startConfig"

#start immediately after startConfig
timeshift_commands(commands[1:],-commands[1]['time'])

print "Sending commands to APC Motion..."
robot = motion.setup(libpath="./",klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob")
print "Starting up..."
res = robot.startup()
print "Start up result:",res
if not res:
    exit(1)

send_command(commands[0],robot)

command = 1
t0 = time.time()
lastMoveTime = t0
skiptime = 0
while command < len(commands) or robot.moving():
    t = time.time()-t0+skiptime
    if robot.moving(): lastMoveTime = t
    if command < len(commands):
        if t >= commands[command]['time']:
            send_command(commands[command],robot)
            command += 1
        elif skip_gaps and (t - lastMoveTime > max_gap):
            #do a skip motion
            send_command(commands[command],robot)
            skiptime = commands[command]['time'] - (time.time()-t0)
            command += 1

print "Done.  Shutting down in 10s, kill me if you want to keep the robot enabled..."
time.sleep(10.0)
robot.shutdown()
