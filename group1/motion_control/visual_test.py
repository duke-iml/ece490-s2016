from klampt import *
from klampt import visualization
from klampt import robotcollide
from klampt import robotcspace
from klampt import cspace
from klampt import resource

world = WorldModel()
world.readFile('klampt_models/apc.xml')

obj = ik.objective(world.robot(0).link(22),local=[0,0,0],world=[1,0,1])

visualization.add("world",world)
#visualization.add("ik objective",obj)
q0 = world.robot(0).getConfig()
q1 = q0[:]
q1[16] = 1
q2 = q1[:]
q2[16] = 0
path = [q0,q1,q2]
q1 = resource.get("goal.config",description="Goal config for left arm",doedit=True,default=q1,editor='visual',world=world)

subset = [15,16,17,18,19,20,21,22]
collider = robotcollide.WorldCollider(world)
#probably want to ignore collisions beteween other arm/links and the world to make things faster...
space = robotcspace.RobotSubsetCSpace(world.robot(0),subset,collider)
planner = cspace.MotionPlan(space, "rrt*")
#extract out cspace configurations
start = [q0[i] for i in subset]
goal = [q1[i] for i in subset]
print "Goal config",goal
planner.setEndpoints(start,goal)
for iters in xrange(10000):
	planner.planMore(1)
	if planner.getPath() != None:
		print "Planning succeeded"
		cspacepath = planner.getPath()	
		#convert back to robot joint space
		path = []
		for qcspace in cspacepath:
			print qcspace
			q = q0[:]
			for i,v in zip(subset,qcspace):
				q[i] = v
			path.append(q)	
		break

visualization.animate("world",path)
visualization.dialog()
visualization.kill()
