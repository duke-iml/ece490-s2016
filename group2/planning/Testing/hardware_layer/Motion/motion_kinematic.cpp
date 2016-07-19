#include "motion_state.h"

class MyControllerUpdateData : public ControllerUpdateData
{
public:
  double lastSensorTime;
  virtual bool MyStartup() {
    if(!robotModel) {
      printf("Motion(Kinematic): Klamp't model is not provided, cannot start\n");
      return false;
    }
    if(!GetKlamptIndices()) {
      //check whether to return false -- accept loss of gripper and base functionality
      for(int i=0;i<numLimbDofs;i++) {
	if(leftKlamptIndices[i] < 0 || rightKlamptIndices[i] < 0) {
	  printf("Motion(Kinematic): Klamp't model does not have appropriate limb indices\n");
	  return false;
	}
      }
      if(headPanKlamptIndex < 0) {
	printf("Motion(Kinematic): Klamp't model does not have appropriate head indices\n");
	return false;
      }
      for(int i=0;i<3;i++) {
	if(baseKlamptIndices[i] < 0) {
	  baseKlamptIndices.resize(0);
	  break;
	}
      }
    }
    if(rightGripperMap.name.empty())
      robotState.rightGripper.enabled = false;
    else {
      robotState.rightGripper.enabled = true;
      robotState.rightGripper.name = rightGripperMap.name;
      int numGripperDofs = (int)rightGripperMap.klamptIndices.size();
      robotState.rightGripper.position.resize(numGripperDofs,0.0);
      robotState.rightGripper.positionCommand.resize(numGripperDofs,0.0);
      robotState.rightGripper.speedCommand.resize(numGripperDofs,0.0);
      robotState.rightGripper.forceCommand.resize(numGripperDofs,0.0);
    }
    if(leftGripperMap.name.empty()) 
      robotState.leftGripper.enabled = false;
    else {
      robotState.leftGripper.enabled = true;
      robotState.leftGripper.name = leftGripperMap.name;
      int numGripperDofs = (int)leftGripperMap.klamptIndices.size();
      robotState.leftGripper.position.resize(numGripperDofs,0.0);
      robotState.leftGripper.positionCommand.resize(numGripperDofs,0.0);
      robotState.leftGripper.speedCommand.resize(numGripperDofs,0.0);
      robotState.leftGripper.forceCommand.resize(numGripperDofs,0.0);
    }
    if(!baseKlamptIndices.empty()) {
      robotState.base.enabled = true;
      robotState.base.odometry.resize(3,0.0);
      robotState.base.velocity.resize(3,0.0);
    }
    robotState.leftLimb.senseUpdateTime = 0;
    robotState.rightLimb.senseUpdateTime = 0;
    KlamptToLimb(planner.world.robots[0].robot->q,LEFT,robotState.leftLimb.sensedConfig);
    KlamptToLimb(planner.world.robots[0].robot->q,RIGHT,robotState.rightLimb.sensedConfig);
    robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
    robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
    KlamptToLimb(planner.world.robots[0].robot->dq,LEFT,robotState.leftLimb.sensedVelocity);
    KlamptToLimb(planner.world.robots[0].robot->dq,RIGHT,robotState.rightLimb.sensedVelocity);
    robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
    robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
    return true;
  }
  virtual bool MyProcessSensors() {
    double dt = t - lastSensorTime;
    //limb sensors
    if(dt > 0) {
      robotState.leftLimb.sensedVelocity = (robotState.leftLimb.commandedConfig - robotState.leftLimb.sensedConfig) / dt;
      robotState.rightLimb.sensedVelocity = (robotState.rightLimb.commandedConfig - robotState.rightLimb.sensedConfig) / dt;
    }
    robotState.leftLimb.sensedConfig = robotState.leftLimb.commandedConfig;
    robotState.rightLimb.sensedConfig = robotState.rightLimb.commandedConfig;
    robotState.leftLimb.senseUpdateTime = t;
    robotState.rightLimb.senseUpdateTime = t;
    lastSensorTime = t;
    return true;
  }
  virtual void MySendCommands() {
    double dt = t - last_t;
    //advance limbs
    if(robotState.leftLimb.sendCommand) {
      if(robotState.leftLimb.controlMode == LimbState::POSITION || robotState.leftLimb.controlMode == LimbState::RAW_POSITION)
	robotState.leftLimb.sendCommand = false;
      else if(robotState.leftLimb.controlMode == LimbState::VELOCITY) 
	robotState.leftLimb.commandedConfig.madd(robotState.leftLimb.commandedVelocity,dt);
      else  //effort mode not supported
	robotState.leftLimb.sendCommand = false;
      //handle joint limits
      if(robotModel) {
	for(size_t i=0;i<leftKlamptIndices.size();i++) {
	  if(robotState.leftLimb.commandedConfig[i] < robotModel->qMin[leftKlamptIndices[i]] ||
	     robotState.leftLimb.commandedConfig[i] > robotModel->qMax[leftKlamptIndices[i]]) {
	    robotState.leftLimb.commandedConfig[i] = Clamp(robotState.leftLimb.commandedConfig[i],robotModel->qMin[leftKlamptIndices[i]],robotModel->qMax[leftKlamptIndices[i]]);
	    robotState.leftLimb.commandedVelocity[i] = 0;
	  }
	}
      }
    }
    if(robotState.rightLimb.sendCommand) {
      if(robotState.rightLimb.controlMode == LimbState::POSITION || robotState.rightLimb.controlMode == LimbState::RAW_POSITION)
	robotState.rightLimb.sendCommand = false;
      else if(robotState.rightLimb.controlMode == LimbState::VELOCITY)
	robotState.rightLimb.commandedConfig.madd(robotState.rightLimb.commandedVelocity,dt);
      else  //effort mode not supported
	robotState.rightLimb.sendCommand = false;
      //handle joint limits
      if(robotModel) {
	for(size_t i=0;i<rightKlamptIndices.size();i++) {
	  if(robotState.rightLimb.commandedConfig[i] < robotModel->qMin[rightKlamptIndices[i]] ||
	     robotState.rightLimb.commandedConfig[i] > robotModel->qMax[rightKlamptIndices[i]]) {
	    robotState.rightLimb.commandedConfig[i] = Clamp(robotState.rightLimb.commandedConfig[i],robotModel->qMin[rightKlamptIndices[i]],robotModel->qMax[rightKlamptIndices[i]]);
	    robotState.rightLimb.commandedVelocity[i] = 0;
	  }
	}
      }
    }
    //advance grippers
    if(robotState.leftGripper.sendCommand) 
      robotState.leftGripper.sendCommand = false;
    robotState.leftGripper.moving = false;
    for(int i=0;i<robotState.leftGripper.position.n;i++) {
      double pdes = Clamp(robotState.leftGripper.positionCommand[i],0.0,1.0);
      double v = Clamp(robotState.leftGripper.speedCommand[i],0.0,1.0);
      double old = robotState.leftGripper.position[i];
      robotState.leftGripper.position[i] += dt*Sign(pdes-robotState.leftGripper.position[i])*v;
      if(Sign(old-pdes) != Sign(robotState.leftGripper.position[i]-pdes))
	//stop
	robotState.leftGripper.position[i] = pdes;
      if(robotState.leftGripper.position[i] != pdes)
	robotState.leftGripper.moving=true;
    }
    if(robotState.rightGripper.sendCommand) 
      robotState.rightGripper.sendCommand = false;
    robotState.rightGripper.moving = false;
    for(int i=0;i<robotState.leftGripper.position.n;i++) {
      double pdes = Clamp(robotState.rightGripper.positionCommand[i],0.0,1.0);
      double v = Clamp(robotState.leftGripper.speedCommand[i],0.0,1.0);
      double old = robotState.rightGripper.position[i];
      robotState.rightGripper.position[i] += dt*Sign(pdes-robotState.rightGripper.position[i])*v;
      if(Sign(old-pdes) != Sign(robotState.rightGripper.position[i]-pdes))
	//stop
	robotState.rightGripper.position[i] = pdes;
      if(robotState.rightGripper.position[i] != pdes)
	robotState.rightGripper.moving = true;
    }
    //TODO: advance base
    if(robotState.base.sendCommand)
      robotState.base.sendCommand = false;
  }
};

#include "motion_common.h"

double getMobileBaseMoveTime()
{
  return 0;
}

///Returns the estimated time until the gripper stops
double getGripperMoveTime(int limb)
{
  return 0;
}

