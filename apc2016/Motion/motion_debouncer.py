from klampt import vectorops
import math


#a debouncer for the Baxter robot that prevents overshoot / bouncing

vmax = 0.2

def smooth_quintic(qcur,vcur,acur,qdes,T):
    b = [qcur-qdes,vcur,acur]
    A = [[T**5,T**4,T**3],
         [5*T**4,4*T**3,3*T**2],
         [20*T**3,12*T**2,6*T]]
    c = np.linalg.inv(np.array(A)).dot(np.array(b))
    print c
    return (lambda t:c[0]*(T-t)**5 + c[1]*(T-t)**4 + c[2]*(T-t)**3 + qdes,
            lambda t:5*c[0]*(T-t)**4 + 4*c[1]*(T-t)**3 + 3*c[2]*(T-t)**2)

def Linf_norm(v):
    return max(abs(x) for x in v)

def append_debounced_command(qend,vend,qdes,motion_queue):
    """Appends a debounced command from qend,vend to qdes,vdes (with vdes=0) 
    to the given motion queue.  Assumes qend,vend is the end of the motion
    queue."""
    global vmax
    #hack: estimate time of motion
    L = Linf_norm(vend) 
    p = vectorops.madd(qend,vend,0.5)
    L += Linf_norm(vectorops.sub(qdes,p))
    T = math.sqrt(L / vmax)
    if T == 0:
        return
    temp = vectorops.madd(qend,vectorops.sub(qdes,qend),0.85)
    motion_queue.appendCubic(T,temp,vectorops.mul(vectorops.sub(qdes,qend),1.0/T*0.25))
    temp2 = vectorops.madd(qend,vectorops.sub(qdes,qend),0.95)
    motion_queue.appendCubic(T*0.75,temp2,vectorops.mul(vectorops.sub(qdes,qend),1.0/T*0.05))
    motion_queue.appendCubic(T*0.5,qdes,[0]*len(qdes))
    return

def send_debounced_command(qcur,vcur,qdes,motion_queue):
    """Sends a debounced command from qcur,vcur to qdes,vdes (with vdes=0) 
    to the given motion queue"""
    global vmax
    #hack: estimate time of motion
    L = Linf_norm(vcur) 
    p = vectorops.madd(qcur,vcur,0.5)
    L += Linf_norm(vectorops.sub(qdes,p))
    T = math.sqrt(L / vmax)
    if T == 0:
        return
    temp = vectorops.madd(qcur,vectorops.sub(qdes,qcur),0.85)
    motion_queue.setCubic(T,temp,vectorops.mul(vectorops.sub(qdes,qcur),1.0/T*0.25))
    temp2 = vectorops.madd(qcur,vectorops.sub(qdes,qcur),0.95)
    motion_queue.appendCubic(T*0.75,temp2,vectorops.mul(vectorops.sub(qdes,qcur),1.0/T*0.05))
    motion_queue.appendCubic(T*0.5,qdes,[0]*len(qdes))
    return

    #old: trying quintic motion
    """
    f = [smooth_quintic(qci,vci,0,qdi,T) for (qci,vci,qdi) in zip(qcur,vcur,qdes)]
    times = []
    milestones = []
    dmilestones = []
    n = math.ceil(T/0.3)
    div = T/n
    for i in range(1,int(n)+1):
        times.append(div)
        milestones.append([fj[0](i*div) for fj in f])
        dmilestones.append([fj[1](i*div) for fj in f])
        if i==1:
            motion_queue.setCubic(div,milestones[-1],dmilestones[-1])
        else:
            if i==int(n):
                motion_queue.appendCubic(div*5,milestones[-1],[0]*len(qcur))
            elif i+1==int(n):
                motion_queue.appendCubic(div,milestones[-1],vectorops.div(dmilestones[-1],5.0))
            else:
                motion_queue.appendCubic(div,milestones[-1],dmilestones[-1])
    """

def send_debounced_motion_command(motion,limb,qdes,append=False):
    """Given a motion library motion, sends the debounced command to the
    given configuration qdes for the indicated limb.  If append=true then
    it is appended to the current motion queue."""
    if limb=='left':
        if append:
            qend = motion.robot.left_mq.target()
            vend = [0]*len(qdes)
            append_debounced_command(qend,vend,qdes,motion.robot.left_mq)
        else:
            qcur = motion.robot.left_limb.commandedPosition()
            vcur = motion.robot.left_limb.commandedVelocity()
            send_debounced_command(qcur,vcur,qdes,motion.robot.left_mq)
    elif limb=='right':
        if append:
            qend = motion.robot.right_mq.target()
            vend = [0]*len(qdes)
            append_debounced_command(qend,vend,qdes,motion.robot.right_mq)
        else:
            qcur = motion.robot.right_limb.commandedPosition()
            vcur = motion.robot.right_limb.commandedVelocity()
            send_debounced_command(qcur,vcur,qdes,motion.robot.right_mq)
    else:
        raise ValueError("Invalid limb "+limb)

