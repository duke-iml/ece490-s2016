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

class Bumper:
    def __init__(self):
        klampt_model = "common/klampt_models/baxter_col.rob"
        mode = "client"

        print "Loading Motion Module model", klampt_model
        motion.setup(mode=mode,klampt_model=klampt_model,libpath="common/",)
        res = motion.robot.startup()
        if not res:
            print "Error starting up Motion Module"
            return
        time.sleep(0.1)
        world = WorldModel()
        res = world.readFile(klampt_model)
        if not res:
            raise RuntimeError("Unable to load Klamp't model", klampt_model)
        #self.robotPoser = RobotPoser(world.robot(0))
        self.kRobot = world.robot(0)
        self.pRobot = motion.robot

    def run(self):
        while True:
            command = raw_input("Command: ")
            l = pRobot.left_limb.sensedPosition()
            r = pRobot.right_limb.sensedPosition()
            q = pRobot.getKlamptSensedPosition()
            kRobot.setConfig(q)
            if command == 'q':
                exit(0)
            elif command == 'c':
                print "right:", getRightArmCoords()
                print "left:", getLeftArmCoords()
            elif command[0] == 'l':
                moveLeftArmTo(make_tuple(command[1:]))
            elif command[0] == 'r':
                moveRightArmTo(make_tuple(command[1:]))
            elif command[0] == 'b':
                coords = make_tuple(command[2:])
                if command[1] =='r':
                    bumpRight(coords)
                elif command[1] =='l':
                    bumpLeft(coords)
                else:
                    print "Unknown command"
            else:
                print "Unknown command"

    def getLeftArmCoords(self):
        return self.kRobot.link("left_wrist").getWorldPosition((0,0,0))

    def getRightArmCoords(self):
        return self.kRobot.link("right_wrist").getWorldPosition((0,0,0))

    def bumpLeft(self, coords):
        config = map(add, coords, getLeftArmCoords())
        moveLeftArmTo(config)

    def bumpRight(self, coords):
        config = map(add, coords, getRightArmCoords())
        moveRightArmTo(config)

    def moveRightArmTo(self, coords):
        iksolve(coords, self.kRobot.link("right_wrist"), self.pRobot.right_limb, self.pRobot.right_mq)

    def moveLeftArmTo(self, coords):
        iksolve(coords, self.kRobot.link("left_wrist"), self.pRobot.left_limb, self.pRobot.left_mq)

    def iksolve(self, config, kEE, pEE, mq):
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
    print "bump.py: bumps the robot arm a specified distance"
    print
    bump = Bumper()
    bump.run()
