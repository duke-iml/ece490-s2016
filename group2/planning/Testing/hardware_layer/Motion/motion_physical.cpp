#include "motion_state.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/HeadState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <reflex_sf_msgs/JointState.h>
#include <reflex_sf_msgs/Command.h>
#include <ros/callback_queue.h>

const static double reflex_offset [4] = { 4.5, 4.5, 4.5, 2.5};
const static double reflex_scaling [4] = { -4.5, -4.5, -4.5, -2.5 };
//const static double reflex_offset [4] = { 0,0,0,0};
//const static double reflex_scaling [4] = { 4.5, 4.5, 4.5, 2.5 };

//50 Hz
const double g_send_command_delay = 0.02;


template <class Msg>
class AutoMsg 
{
public:
  AutoMsg(): msgCount(0),lastReadMsg(0) {}
  void subscribe(ros::NodeHandle& nh,const char* topic,int queueSize = 100) {
    this->topic = topic;
    sub = nh.subscribe(topic,10,&AutoMsg<Msg>::callback, this);
  }
  void unsubscribe() {
    this->topic = "";
    sub = ros::Subscriber();
  }
  ///Use this or numUnread() to tell if a new message arrived since the last read
  bool changed() const { return msgCount != lastReadMsg; }
  ///Use this to tell how many new messages arrived since the last read
  int numUnread() const { return msgCount - lastReadMsg; }
  ///Use this to tell if a new message arrived
  void markRead() { lastReadMsg = msgCount; }

  void callback(const Msg& msg) { 
    value = msg; msgCount++; 
  }
  operator const Msg&  () const { return value; }
  operator Msg& () { return value; }
  const Msg* operator ->  () const { return &value; }
  Msg* operator -> () { return &value; }

  ros::Subscriber sub;
  string topic;
  Msg value;
  int msgCount;
  int lastReadMsg;
};

const char* baxter_left_limb_names [] = { "left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2","left_w3"};
const char* baxter_right_limb_names [] = { "right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2","right_w3"};

class MyControllerUpdateData : public ControllerUpdateData
{
public:
  //ROS stuff
  SmartPointer<ros::NodeHandle> nh;
  ros::Publisher larm_pub,rarm_pub,head_pan_pub,head_nod_pub,left_gripper_pub,right_gripper_pub,base_pub;
  double last_send_time;
  AutoMsg<baxter_core_msgs::AssemblyState> assemblyState;
  AutoMsg<sensor_msgs::JointState> jointState;
  AutoMsg<baxter_core_msgs::HeadState> headState;
  AutoMsg<baxter_core_msgs::EndEffectorState> leftGripperState,rightGripperState;
  AutoMsg<reflex_sf_msgs::JointState> leftReflexGripperState,rightReflexGripperState;
  AutoMsg<nav_msgs::Odometry> baseState;

  //other physical robot state
  bool baxterEstop;

  map<string,int> baxter_left_limb_map,baxter_right_limb_map;

  MyControllerUpdateData()
    :last_send_time(-1),baxterEstop(false)
  {}

  virtual bool MyStartup();
  virtual bool MyProcessSensors();
  virtual void MySendCommands();
  virtual void MyShutdown();
};

#include "motion_common.h"


bool MyControllerUpdateData::MyStartup() {
  if(robotModel) {
    if(!GetKlamptIndices()) {
      // printf("Will proceed without some Klamp't indices\n");
    }
  }
  char* argv [] = { "apc_motion" };
  int argc=1;
  //ros::init(argc, &argv[0], "apc_motion", ros::init_options::NoSigintHandler);
  ros::init(argc, &argv[0], "apc_motion");

  //set up map from ros names to indices
  for(int i=0;i<numLimbDofs;i++)
    baxter_left_limb_map[baxter_left_limb_names[i]]=i;
  for(int i=0;i<numLimbDofs;i++)
    baxter_right_limb_map[baxter_right_limb_names[i]]=i;

  nh = new ros::NodeHandle();
  larm_pub = nh->advertise<baxter_core_msgs::JointCommand>(string("robot/limb/left/joint_command"),10,false);
  rarm_pub = nh->advertise<baxter_core_msgs::JointCommand>(string("robot/limb/right/joint_command"),10,false);
  head_pan_pub = nh->advertise<baxter_core_msgs::HeadPanCommand>(string("robot/head/command_head_pan"),100,false);
  head_nod_pub = nh->advertise<std_msgs::Bool>("robot/head/command_head_nod",10,false);
  base_pub = nh->advertise<geometry_msgs::Twist>("base/output/cmd/ts",10,false);
  
  assemblyState.subscribe(*nh,"robot/state",10);
  jointState.subscribe(*nh,"robot/joint_states",10);
  headState.subscribe(*nh,"robot/head/head_state",10);
  leftGripperState.subscribe(*nh,"robot/end_effector/left_gripper/state",10);
  rightGripperState.subscribe(*nh,"robot/end_effector/right_gripper/state",10);

  //try to subscribe to ReFlex hand status messages
  const string leftPrefix = "reflex_hand1";
  const string rightPrefix = "reflex_hand2";
  leftReflexGripperState.subscribe(*nh,(leftPrefix+string("/state")).c_str(),10);
  rightReflexGripperState.subscribe(*nh,(rightPrefix+string("/state")).c_str(),10);

  baseState.subscribe(*nh,"global_odom_pose");
  
  //DO STARTUP
  ros::Publisher enabler = nh->advertise<std_msgs::Bool>("robot/set_super_enable",10);
  ros::Publisher resetter = nh->advertise<std_msgs::Empty>("robot/set_super_reset",10);
  //wait for robot to be enabled
  while(!kill && ros::ok()) {
    if(assemblyState.msgCount > 0 && assemblyState->enabled) {
      if(assemblyState->stopped) {
	printf("Robot ESTOP is on!  Turn off EStop and manually reset.\n");
	resetter.publish(std_msgs::Empty());
      }
      else {
	// printf("Robot is enabled and ready to run\n");
	break;
      }
    }
    else {
      std_msgs::Bool msg;
      msg.data = true;
      enabler.publish(msg);
      printf("Waiting for robot to be enabled...\n");
    }
    ros::spinOnce();
    ThreadSleep(0.5);
  }
  if(kill || !ros::ok()) return false;
  //wait for robot joint and head messages
  while(!kill && ros::ok()) {
    if(jointState.msgCount > 0 && headState.msgCount > 0) {
      MyProcessSensors();
      break;
    }
    printf("Waiting for joint state and head update messages...\n");
    ros::spinOnce();
    ThreadSleep(0.5);
  }
  if(kill || !ros::ok()) return false;

  //test for gripper connection and enable / calibrate if available
  while(!kill && ros::ok()) {
    if(leftGripperState.msgCount > 0 && robotState.leftGripper.enabled == 0) {
      if(leftGripperState->enabled == 1) {
	if(leftGripperState->calibrated == 2) //not attached
	  robotState.leftGripper.enabled = 0;
	else {
	  robotState.leftGripper.enabled = 1;
	  robotState.leftGripper.name = "Rethink Electric Gripper";
	  robotState.leftGripper.positionCommand.resize(1);
	  robotState.leftGripper.speedCommand.resize(1,0);
	  robotState.leftGripper.forceCommand.resize(1,0);
	  robotState.leftGripper.positionCommand[0] = leftGripperState->position/100.0;
	  left_gripper_pub = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command",10,false);
	  if(leftGripperState->calibrated != 1) {
	    baxter_core_msgs::EndEffectorCommand msg;
	    msg.id = rightGripperState->id;
	    msg.command = "clear_calibration";
	    left_gripper_pub.publish(msg);
	    ros::spinOnce();
	    ThreadSleep(0.5);

	    msg.id = leftGripperState->id;
	    msg.command = "calibrate";
	    left_gripper_pub.publish(msg);
	    printf("Sending left gripper calibrate message\n");
	  }
	}
      }
      else {
	robotState.leftGripper.enabled = 0;
      }
    }
    if(rightGripperState.msgCount > 0 && robotState.rightGripper.enabled == 0) {
      if(rightGripperState->enabled == 1) {
	if(rightGripperState->calibrated == 2) { //not attached
	  robotState.rightGripper.enabled = 0;
	}
	else {
	  robotState.rightGripper.enabled = 1;
	  robotState.rightGripper.name = "Rethink Electric Gripper";
	  robotState.rightGripper.positionCommand.resize(1,0);
	  robotState.rightGripper.speedCommand.resize(1,0);
	  robotState.rightGripper.forceCommand.resize(1,0);
	  robotState.rightGripper.positionCommand[0] = rightGripperState->position/100.0;
	  right_gripper_pub = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command",10,false);
	  if(rightGripperState->calibrated != 1) {
	    baxter_core_msgs::EndEffectorCommand msg;
	    msg.id = rightGripperState->id;
	    msg.command = "clear_calibration";
	    right_gripper_pub.publish(msg);
	    ros::spinOnce();
	    ThreadSleep(0.5);
	    
	    msg.id = rightGripperState->id;
	    msg.command = "calibrate";
	    right_gripper_pub.publish(msg);
	    printf("Sending right gripper calibrate message\n");
	  }
	}
      }
      else {
	robotState.rightGripper.enabled = 0;
      }
    }
    if(leftReflexGripperState.msgCount > 0) {
      // printf("Got Reflex gripper on left hand\n");
      robotState.leftGripper.enabled = 1;
      robotState.leftGripper.name = "ReFlex";
      robotState.leftGripper.positionCommand.resize(4,0);
      robotState.leftGripper.speedCommand.resize(4,0);
      robotState.leftGripper.forceCommand.resize(4,0);
      for(int i=0;i<4;i++)
	robotState.leftGripper.positionCommand[i] = (leftReflexGripperState->current_pos[i]-reflex_offset[i])/reflex_scaling[i];
      left_gripper_pub = nh->advertise<reflex_sf_msgs::Command>("reflex_hand1/command",10,false);
      //unsubscribe from Baxter messages
      leftGripperState.unsubscribe();
    }
    if(rightReflexGripperState.msgCount > 0) {
      // printf("Got Reflex gripper on right hand\n");
      robotState.rightGripper.enabled = 1;
      robotState.rightGripper.name = "ReFlex";
      robotState.rightGripper.positionCommand.resize(4,0);
      robotState.rightGripper.speedCommand.resize(4,0);
      robotState.rightGripper.forceCommand.resize(4,0);
      for(int i=0;i<4;i++)
	robotState.rightGripper.positionCommand[i] = (rightReflexGripperState->current_pos[i]-reflex_offset[i])/reflex_scaling[i];
      right_gripper_pub = nh->advertise<reflex_sf_msgs::Command>("reflex_hand2/command",10,false);
      //unsubscribe from Baxter messages
      rightGripperState.unsubscribe();
    }
    if(leftReflexGripperState.msgCount > 0 && rightReflexGripperState.msgCount > 0) {
      //done waiting for grippers
      printf("Done waiting for grippers (got reflex messages)\n");
      break;
    }
    if(leftGripperState.msgCount > 0 && rightGripperState.msgCount > 0) {
      if((leftGripperState->enabled == 0 || leftGripperState->calibrated==2 || leftGripperState->calibrated == 1) && (rightGripperState->enabled == 0 || rightGripperState->calibrated==2 || rightGripperState->calibrated == 1)) {
	//done waiting for grippers
	// printf("Done waiting for grippers\n");
	break;
      }
    }
    printf("Waiting for gripper messages and gripper calibration...\n");
    if(leftGripperState.msgCount > 0)
      printf("Left gripper enabled: %d, calibrated %d\n",leftGripperState->enabled,leftGripperState->calibrated);
    if(rightGripperState.msgCount > 0)
      printf("Right gripper enabled: %d, calibrated %d\n",rightGripperState->enabled,rightGripperState->calibrated);
    ros::spinOnce();
    ThreadSleep(0.5);
  }
  if(kill || !ros::ok()) return false;

  //test for base connection and enable if available
  Timer timer;
  // printf("Waiting for base messages...\n");
  while(!kill && ros::ok() && timer.ElapsedTime() < 0.5) {
    if(baseState.msgCount > 0) 
      robotState.base.enabled = 1;
    ros::spinOnce();
    ThreadSleep(0.01);    
  }
  // if(robotState.base.enabled)
    // printf("Base is enabled.\n");
  // else
    // printf("Base is not enabled.\n");
  return true;
}


bool MyControllerUpdateData::MyProcessSensors() 
{
  bool processed = false;
  //ros::spinOnce();
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
  if(!ros::ok()) { 
    kill = true;
    return false;
  }
  if(baxterEstop) {
    kill = true;
    return false;
  }
  ScopedLock lock(mutex);
  if(assemblyState.changed()) {
    baxterEstop = assemblyState->stopped;
    assemblyState.markRead();
  }
  if(jointState.changed()) {
    //update left and right arm states
    jointState.markRead();
    robotState.leftLimb.sensedConfig.resize(numLimbDofs);
    robotState.leftLimb.sensedVelocity.resize(numLimbDofs);
    robotState.leftLimb.sensedEffort.resize(numLimbDofs);
    robotState.rightLimb.sensedConfig.resize(numLimbDofs);
    robotState.rightLimb.sensedVelocity.resize(numLimbDofs);
    robotState.rightLimb.sensedEffort.resize(numLimbDofs);
    robotState.leftLimb.senseUpdateTime = t;
    robotState.rightLimb.senseUpdateTime = t;
    for(size_t i=0;i<jointState->name.size();i++) {
      if(baxter_left_limb_map.count(jointState->name[i]) != 0) {
	int k=baxter_left_limb_map[jointState->name[i]];
	robotState.leftLimb.sensedConfig[k] = jointState->position[i];
	robotState.leftLimb.sensedVelocity[k] = jointState->velocity[i];
	robotState.leftLimb.sensedEffort[k] = jointState->effort[i];
      }
      else if(baxter_right_limb_map.count(jointState->name[i]) != 0) {
	int k=baxter_right_limb_map[jointState->name[i]];
	robotState.rightLimb.sensedConfig[k] = jointState->position[i];
	robotState.rightLimb.sensedVelocity[k] = jointState->velocity[i];
	robotState.rightLimb.sensedEffort[k] = jointState->effort[i];
      }      
    }
    if(robotState.leftLimb.commandedConfig.empty()) {
      robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
      robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
      robotState.leftLimb.commandedEffort = robotState.leftLimb.sensedEffort;
    }
    if(robotState.rightLimb.commandedConfig.empty()) {
      robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
      robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
      robotState.rightLimb.commandedEffort = robotState.rightLimb.sensedEffort;
    }
  }
  if(headState.changed()) {
    //update head states
    robotState.head.pan = headState->pan;
    robotState.head.isNodding = headState->isNodding;
    headState.markRead();
  }
  if(leftGripperState.changed()) {
    //update gripper states
    if(leftGripperState->moving==1)
      robotState.leftGripper.moving = true;
    else
      robotState.leftGripper.moving = false;
    if(leftGripperState->enabled && leftGripperState->calibrated==1)
      robotState.leftGripper.enabled = true;
    else
      robotState.leftGripper.enabled = false;
    robotState.leftGripper.position.resize(1);
    robotState.leftGripper.force.resize(1);
    robotState.leftGripper.position[0] = leftGripperState->position/100.0;
    robotState.leftGripper.force[0] = leftGripperState->force/100.0;
    leftGripperState.markRead();
  }
  if(rightGripperState.changed()) {
    //update gripper states
    if(rightGripperState->moving==1) 
      robotState.rightGripper.moving = true;
    else
      robotState.rightGripper.moving = false;
    if(rightGripperState->enabled && rightGripperState->calibrated==1)
      robotState.rightGripper.enabled = true;
    else
      robotState.rightGripper.enabled = false;
    robotState.rightGripper.position.resize(1);
    robotState.rightGripper.force.resize(1);
    robotState.rightGripper.position[0] = rightGripperState->position/100.0;
    robotState.rightGripper.force[0] = rightGripperState->force/100.0;
    rightGripperState.markRead();
  }
  
  if(leftReflexGripperState.changed())  {
    robotState.leftGripper.moving = leftReflexGripperState->is_moving;
    robotState.leftGripper.position.resize(4);
    //robotState.leftGripper.speed.resize(4,0);
    robotState.leftGripper.force.resize(4);
    for(int i=0;i<4;i++) {
      robotState.leftGripper.position[i] = (leftReflexGripperState->current_pos[i]-reflex_offset[i])/reflex_scaling[i];
      //robotState.leftGripper.speed[i] = leftReflexGripperState->velocity[i];
      robotState.leftGripper.force[i] = leftReflexGripperState->load[i];
    }
    leftReflexGripperState.markRead();
  }
  if(rightReflexGripperState.changed())  {
    robotState.rightGripper.moving = rightReflexGripperState->is_moving;
    robotState.rightGripper.position.resize(4);
    //robotState.rightGripper.speed.resize(4,0);
    robotState.rightGripper.force.resize(4);
    for(int i=0;i<4;i++) {
      robotState.rightGripper.position[i] = (rightReflexGripperState->current_pos[i]-reflex_offset[i])/reflex_scaling[i];
      //robotState.rightGripper.speed[i] = rightReflexGripperState->velocity[i];
      robotState.rightGripper.force[i] = rightReflexGripperState->load[i];
    }
    rightReflexGripperState.markRead();
  }
  return true;
}

void MyControllerUpdateData::MySendCommands()
{
  double dt = t-last_t;
  //control rate here
  if(t - last_send_time < g_send_command_delay) return;
  last_send_time = t;
  if(robotState.leftLimb.sendCommand || !robotState.leftLimb.rawCommandToSend.empty()) {
    baxter_core_msgs::JointCommand msg;
    msg.names.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) msg.names[i] = baxter_left_limb_names[i];
    mutex.lock();
    const Config* x=NULL;
    if(robotState.leftLimb.controlMode == LimbState::POSITION) {
      //x=&robotState.leftLimb.commandedConfig;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::RAW_POSITION) {
      //x=&robotState.leftLimb.commandedConfig;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::VELOCITY) {
      //x=&robotState.leftLimb.commandedVelocity;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::EFFORT) {
      x=&robotState.leftLimb.commandedEffort;
      //update command feedback from sensors??
      robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
      robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
      msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    }
    robotState.leftLimb.sendCommand = false;

    if(x) {
      msg.command.resize(x->n);
      for(int i=0;i<x->n;i++)
	msg.command[i] = (*x)(i);
      mutex.unlock();
      larm_pub.publish(msg);
    }
    else
      mutex.unlock();
  }
  if(robotState.rightLimb.sendCommand  || !robotState.rightLimb.rawCommandToSend.empty()) {
    baxter_core_msgs::JointCommand msg;
    msg.names.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) msg.names[i] = baxter_right_limb_names[i];

    mutex.lock();
    const Config* x=NULL;
    if(robotState.rightLimb.controlMode == LimbState::POSITION) {
      //x=&robotState.rightLimb.commandedConfig;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::RAW_POSITION) {
      //x=&robotState.rightLimb.commandedConfig;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::VELOCITY) {
      //x=&robotState.rightLimb.commandedVelocity;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::EFFORT) {
      x=&robotState.rightLimb.commandedEffort;
      //update command feedback from sensors??
      robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
      robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
      msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    }
    robotState.rightLimb.sendCommand = false;
    if(x) {
      msg.command.resize(x->n);
      for(int i=0;i<x->n;i++)
	msg.command[i] = (*x)(i);
      mutex.unlock();
      rarm_pub.publish(msg);
    }
    else
      mutex.unlock();
  }
  if(robotState.head.sendCommand) {
    ScopedLock lock(mutex);
    baxter_core_msgs::HeadPanCommand msg;
    msg.target = robotState.head.panTarget;
    msg.speed = robotState.head.panSpeed;
    head_pan_pub.publish(msg);
    robotState.head.sendCommand = false;
  }
  if(robotState.base.sendCommand) {
    ///TODO: base motion
    robotState.base.sendCommand = false;
  }
  if(robotState.leftGripper.sendCommand) {
    if(robotState.leftGripper.position.size()==1) { //electric gripper
      ScopedLock lock(mutex);
      ///TODO: gripper force and velocity control
      baxter_core_msgs::EndEffectorCommand msg;
      msg.id = leftGripperState->id;
      {
	stringstream ss;
	ss<<"{\"velocity\":"<<robotState.leftGripper.speedCommand[0]*100<<",\"moving_force\":"<<robotState.leftGripper.forceCommand[0]*110<<",\"holding_force\":"<<robotState.leftGripper.forceCommand[0]*100<<",\"dead_zone\": 5.0 }";
	msg.command = "configure";
	msg.args = ss.str();
	//left_gripper_pub.publish(msg);
      }
      {
	msg.command = "go";
	stringstream ss;
	ss<<"{\"position\":"<<robotState.leftGripper.positionCommand[0]*100<<"}";
	
	msg.args = ss.str();
	left_gripper_pub.publish(msg);
      }
    }
    else if(robotState.leftGripper.position.size()==4) {  //reflex gripper
      ScopedLock lock(mutex);
      reflex_sf_msgs::Command cmd;
      cmd.position.resize(4);
      cmd.force.resize(4);
      for(int i=0;i<4;i++) {
	cmd.position[i] = reflex_offset[i]+reflex_scaling[i]*robotState.leftGripper.positionCommand[i];
	cmd.force[i] = robotState.leftGripper.forceCommand[i];
      }
      left_gripper_pub.publish(cmd);
    }
    robotState.leftGripper.sendCommand = false;
  }
  if(robotState.rightGripper.sendCommand) {
    if(robotState.rightGripper.position.size()==1) { //electric gripper
      ScopedLock lock(mutex);
      baxter_core_msgs::EndEffectorCommand msg;
      msg.id = rightGripperState->id;
      {
	stringstream ss;
	ss<<"{\"velocity\":"<<robotState.rightGripper.speedCommand[0]*100<<",\"moving_force\":"<<robotState.rightGripper.forceCommand[0]*110<<",\"holding_force\":"<<robotState.rightGripper.forceCommand[0]*100<<",\"dead_zone\": 5.0 }";
	msg.command = "configure";
	msg.args = ss.str();
	//right_gripper_pub.publish(msg);
      }
      {
	msg.command = "go";
	stringstream ss;
	ss<<"{\"position\":"<<robotState.rightGripper.positionCommand[0]*100<<"}";
	msg.args = ss.str();
	right_gripper_pub.publish(msg);
      }
    }
    else if(robotState.rightGripper.position.size()==4) {  //reflex gripper
      ScopedLock lock(mutex);
      reflex_sf_msgs::Command cmd;
      cmd.position.resize(4);
      cmd.force.resize(4);
      for(int i=0;i<4;i++) {
	cmd.position[i] = reflex_offset[i] + reflex_scaling[i]*robotState.rightGripper.positionCommand[i];
	cmd.force[i] = robotState.rightGripper.forceCommand[i];
      }
      right_gripper_pub.publish(cmd);
    }
    robotState.rightGripper.sendCommand = false;
  }
  ros::spinOnce();
}

void MyControllerUpdateData::MyShutdown()
{
  ros::Publisher enabler = nh->advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  while(ros::ok()) {
    if(!assemblyState->enabled) {
      printf("Successful shutdown, robot is now disabled\n");
      break;
    }
    else {
      std_msgs::Bool msg;
      msg.data = false;
      enabler.publish(msg);
    }
    ros::spinOnce();
    ThreadSleep(0.5);
  }

  ros::shutdown();
}


double getMobileBaseMoveTime()
{
  return 0;
}

///Returns the estimated time until the gripper stops
double getGripperMoveTime(int limb)
{
  return 0;
}

