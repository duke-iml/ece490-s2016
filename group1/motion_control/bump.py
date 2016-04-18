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

##
# @brief Bumper a class that will bump the robot arm to a point
# in space relative to world coordinates or current end effector
# coordinates
##
class Bumper:
    ##
    # @brief Constructor
    #
    def __init__(self):
        klampt_model = "common/klampt_models/baxter_col.rob"
        mode = "client"

        print "Loading Motion Module model", klampt_model
        motion.setup(mode=mode,klampt_model=klampt_model,libpath="common/",)
        res = motion.robot.startup()
        if not res:
            raise RuntimeError("Unable to startup motion module")
        time.sleep(0.1)
        self.world = WorldModel()
        res = self.world.readFile(klampt_model)
        if not res:
            raise RuntimeError("Unable to load Klamp't model", klampt_model)
        self.kRobot = self.world.robot(0)
        self.pRobot = motion.robot

    ##
    # @brief Returns the coordinates of the left arm (x, y, z)
    #
    # @return the coordinates
    ##
    def getLeftArmCoords(self):
        self.setKlamptPos()
        return self.kRobot.link("left_wrist").getWorldPosition((0,0,0))

    ##
    # @brief Returns the coordinates of the right arm (x, y, z)
    #
    # @return the coordinates
    ##
    def getRightArmCoords(self):
        self.setKlamptPos()
        return self.kRobot.link("right_wrist").getWorldPosition((0,0,0))

    ##
    # @brief Bumps the left arm to the specified coordinates (in end effector coordinates)
    #
    # This function is the basis of the bumper and is responsible for moving the end effector
    # a specific amount in a specified direction, given by the coords parameter
    #
    # @param The coordinates to move to (in relative coordinates)
    ##
    def bumpLeft(self, coords):
        config = map(add, coords, self.getLeftArmCoords())
        self.moveLeftArmTo(config)

    ##
    # @brief Bumps the right arm to the specified coordinates (in end effector coordinates)
    #
    # This function is the basis of the bumper and is responsible for moving the end effector
    # a specific amount in a specified direction, given by the coords parameter
    #
    # @param The coordinates to move to (in relative coordinates)
    ##
    def bumpRight(self, coords):
        config = map(add, coords, self.getRightArmCoords())
        self.moveRightArmTo(config)

    ##
    # @brief Moves the right arm to the specified coordinates
    #
    # @param The coordinates to move to
    ##
    def moveRightArmTo(self, coords):
        self.iksolve(coords, self.kRobot.link("right_wrist"), self.pRobot.right_limb, self.pRobot.right_mq)

    ##
    # @brief Moves the left arm to the specified coordinates
    #
    # @param The coordinates to move to
    ##
    def moveLeftArmTo(self, coords):
        self.iksolve(coords, self.kRobot.link("left_wrist"), self.pRobot.left_limb, self.pRobot.left_mq)

    ##
    # @brief Performs IK on a position and moves baxter there
    #
    # @param config The world coordinates to perform IK on
    # @param kEE The Kinematic end effector
    # @param pEE The Physical end effector
    # @param mq The motion queue for the particular end effector
    ##
    def iksolve(self, config, kEE, pEE, mq):
        goal = ik.objective(kEE,local=(0,0,0), world=config)
        self.setKlamptPos()
        if ik.solve(goal):
            print "success!"
            q = self.kRobot.getConfig()
            mq.setRamp(pEE.configFromKlampt(q))
        else:
            print "failed. Residual:", ik.solver(goal).getResidual()

    ##
    # @brief Gets the robots sensed position and set's klampts model to reflect that
    ##
    def setKlamptPos(self):
        q = self.pRobot.getKlamptSensedPosition()
        self.kRobot.setConfig(q)

##
# @brief Command line tester function
#
# @param bump a Bumper object
##
def run(bump):
        while True:
            command = raw_input("Command: ")
            if command == 'q':
                exit(0)
            elif command == 'c':
                print "right:", bump.getRightArmCoords()
                print "left:", bump.getLeftArmCoords()
            elif command[0] == 'l':
                bump.moveLeftArmTo(make_tuple(command[1:]))
            elif command[0] == 'r':
                bump.moveRightArmTo(make_tuple(command[1:]))
            elif command[0] == 'b':
                coords = make_tuple(command[2:])
                if command[1] =='r':
                    bump.bumpRight(coords)
                elif command[1] =='l':
                    bump.bumpLeft(coords)
                else:
                    print "Unknown command"
            else:
                print "Unknown command"

##
# @brief Main method.  Creates a Bumper and runs the test method run()
##
if __name__ == "__main__":
    global robotPoser
    print "bump.py: bumps the robot arm a specified distance"
    print
    bump = Bumper()
    run(bump)
