#!/usr/bin/python2.7
from klampt import *
from klampt import RobotPoser, ik, se3
import time
from common.Motion import motion
from ast import literal_eval as make_tuple
from operator import add

# motion.robot.left_limb = Limb(LEFT)
# motion.robot.right_limb = Limb(RIGHT)
# motion.robot.left_ee = EndEffector(LEFT)
# motion.robot.right_ee = EndEffector(RIGHT)
# motion.robot.left_gripper = Gripper(LEFT)
# motion.robot.right_gripper = Gripper(RIGHT)
# motion.robot.head = Head()
# motion.robot.base = MobileBase()
# motion.robot.left_mq = LimbMotionQueue(LEFT)
# motion.robot.right_mq = LimbMotionQueue(RIGHT)
# motion.robot.arms_mq = LimbMotionQueue(BOTH)
# motion.robot.planner = Planner()
# motion.robot.left_planner = LimbPlanner(LEFT)
# motion.robot.right_planner = LimbPlanner(RIGHT)
# motion.robot.arms_planner = LimbPlanner(BOTH)

klampt_model = "common/klampt_models/baxter_col.rob"
mode = "client"

def run():
    global kRobot
    global pRobot
    kRobot = world.robot(0)
    lWrist = kRobot.link("left_wrist")
    rWrist = kRobot.link("right_wrist")
    pRobot = motion.robot
    while True:
        command = raw_input("Command: ")
        l = pRobot.left_limb.sensedPosition()
        r = pRobot.right_limb.sensedPosition()
        q = pRobot.getKlamptSensedPosition()
        kRobot.setConfig(q)
        if command == 'q':
            exit(0)
        elif command == 'c':
            print "right:", lWrist.getWorldPosition((0,0,0))
            print "left:", rWrist.getWorldPosition((0,0,0))
        elif command[0] == 'l':
            config = make_tuple(command[1:])
            iksolve(config, lWrist, pRobot.left_limb, pRobot.left_mq)
        elif command[0] == 'r':
            config = make_tuple(command[1:])
            iksolve(config, rWrist, pRobot.right_limb, pRobot.right_mq)
        elif command[0] == 'b':
            if command[1] =='r':
                config = map(add, make_tuple(command[2:]), rWrist.getWorldPosition((0,0,0)))
                print "new:", config
                iksolve(config, rWrist, pRobot.right_limb, pRobot.right_mq)
            elif command[1] =='l':
                config = map(add, make_tuple(command[2:]), lWrist.getWorldPosition((0,0,0)))
                print "new:", config
                iksolve(config, lWrist, pRobot.left_limb, pRobot.left_mq)
            else:
                print "Unknown command"
        else:
            print "Unknown command"

def iksolve(config, kEE, pEE, mq):
    goal = ik.objective(kEE,local=(0,0,0), world=config)
    q = pRobot.getKlamptSensedPosition()
    kRobot.setConfig(q)
    if ik.solve(goal):
        print "success!"
        q = kRobot.getConfig()
        mq.setRamp(pEE.configFromKlampt(q))
    else:
        print "failed. Residual:", ik.solver(goal).getResidual()

if __name__ == "__main__":
    global robotPoser
    print "widget_control.py: manually sends configurations to the Motion module"
    print

    print "Loading Motion Module model",klampt_model
    motion.setup(mode=mode,klampt_model=klampt_model,libpath="common/",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model", klampt_model)
    robotPoser = RobotPoser(world.robot(0))
    run()
