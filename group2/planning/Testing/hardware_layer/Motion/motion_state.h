#include "motion.h"
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include "Interface/RobotInterface.h"
#include <math/sparsevector.h>
#include <utils/AnyCollection.h>
#include <IO/XmlWorld.h>
#include <robotics/Rotation.h>
#include <robotics/IKFunctions.h>
#include <math/angle.h>
#include <math/SVDecomposition.h>
#include <utils/stringutils.h>
//#include <sspp/Topic.h>
//#include <sspp/Send.h>
#include <fstream>

const char* klampt_left_limb_names [] = {
  "left_upper_shoulder",
  "left_lower_shoulder",
  "left_upper_elbow",
  "left_lower_elbow",
  "left_upper_forearm",
  "left_lower_forearm",
  "left_wrist"
};


const char* klampt_right_limb_names [] = {
  "right_upper_shoulder",
  "right_lower_shoulder",
  "right_upper_elbow",
  "right_lower_elbow",
  "right_upper_forearm",
  "right_lower_forearm",
  "right_wrist"
};


const char* klampt_left_ee_name = "left_gripper";

const char* klampt_right_ee_name = "right_gripper";

const char* klampt_left_gripper_base_name = "left_gripper:base";

const char* klampt_right_gripper_base_name = "right_gripper:base";

const char* klampt_head_name = "head";

const char* klampt_base_names [] = {
  "base_x",
  "base_y",
  "base_theta"
};


struct ControllerUpdateData;

//Typedef MyControllerUpdateData to be your own control loop

struct HeadState
{
  HeadState() { pan=panTarget=panSpeed=0; isNodding=false; sendCommand=false;}
  double pan;
  bool isNodding;

  double panTarget,panSpeed;
  bool sendCommand;
};

struct LimbState
{
  LimbState(int _limb) { limb=_limb; controlMode=NONE; enableSelfCollisionAvoidance=true; senseUpdateTime=-1; sendCommand=false; governor = Normal; motionQueueActive=false; }
  void Advance(Real dt,ControllerUpdateData* context);

  int limb;

  enum {Normal,EndEffectorDrive,MotionQueue,Planner};
  int governor;

  enum {NONE,POSITION,VELOCITY,EFFORT,RAW_POSITION};
  int controlMode;
  bool enableSelfCollisionAvoidance;

  double senseUpdateTime;
  Vector sensedConfig,sensedVelocity,sensedEffort;
  Vector commandedConfig,commandedVelocity,commandedEffort;
  Vector rawCommandToSend;
  bool sendCommand;
  
  Vector integralError;

  //used the governor is EndEffectorDrive (end effector drive command)
  RigidTransform driveTransform,achievedTransform;
  Vector3 driveAngVel,driveVel;

  //used when the governor is MotionQueue or Planner
  bool motionQueueActive;
  PolynomialMotionQueue motionQueue;
};

struct BaseState
{
  BaseState() { controlMode=NONE; enabled=false; moving=false; sendCommand=false; }
  bool GetOdometryTarget(Vector& odo) const;
  bool GetRelativeTarget(Vector& trel) const;
  void ToLocal(const Vector& odoconfig,Vector& lconfig) const;
  void ToOdometry(const Vector& lconfig,Vector& odoconfig) const;
  bool enabled;
  bool moving;
  Vector odometry,velocity;

  enum {NONE,RELATIVE_POSITION,ODOMETRY_POSITION,RELATIVE_VELOCITY};
  int controlMode;  
  Vector command;
  bool sendCommand;
};

struct GripperState
{
  GripperState() { enabled=false; name="none"; moving=false; sendCommand=false; }
  bool enabled;
  string name;
  bool moving;
  Vector position,force;

  bool sendCommand;
  Vector positionCommand,speedCommand,forceCommand;
};

struct PlannerState
{
  enum PlannerType { None, IKPlanner };

  PlannerState() { plannerActive.resize(2,false);  currentPlanner=None; }

  //whether the planner is active for the given limb
  vector<bool> plannerActive;
  RealTimePlanningThread planningThread;
  RobotWorld world;
  WorldPlannerSettings settings;
  SmartPointer<SingleRobotCSpace> cspace;
  PolynomialMotionQueue wholeRobotMotionQueue;
  SmartPointer<DefaultMotionQueueInterface> motionQueueInterface;
  PlannerType currentPlanner;
};

struct RobotState
{
  RobotState():baxterEstop(false),baseEstop(false),leftLimb(LEFT),rightLimb(RIGHT) {}
  bool baxterEstop,baseEstop;
  HeadState head;
  BaseState base;
  LimbState leftLimb,rightLimb;
  GripperState leftGripper,rightGripper;
};

struct KlamptGripperMap
{
  //stores a mapping from command variables to klampt variables
  //each command variable i has a basis function q[li] = si*u+oi
  //where li is the list of links, si is the vector of scale coefficients
  //and oi is the vector of offset coefficients.
  void CommandToConfig(const double* cmd,double* q) const;
  void ConfigToCommand(const double* q,double* cmd) const;
  void FingerToConfig(int index,double value,double* q) const;
  double ConfigToFinger(const double* q,int index) const;

  string name;
  vector<vector<int> > klamptIndices;
  vector<vector<double> > klamptScaleCoefs;
  vector<vector<double> > klamptOffsetCoefs;
};

class BaxterMotorCalibration
{
public:
  BaxterMotorCalibration();
  ///Returns true if it's loaded
  bool Enabled() const;
  ///clears the calibration
  void Clear();
  ///load from the calibration JSON file
  bool Load(const char* fn,const char* limb);
  ///Computes the command for the given q,v
  void ToCommand(const Vector& q,const Vector& v,const Vector& g,
		 const vector<Vector>& qdesired,Vector& qcmd);

  string name;
  int lookahead;
  Real rate;
  vector<SparseVector> Brows,Crows;
  Vector d;
};


///Subclasses will fill this in with your own low-level controller data.
///Make sure to lock the mutex properly!
struct ControllerUpdateData
{
  ControllerUpdateData()
    :startup(false),failure(false),running(false),kill(false)
  {}
  //Klamp't interface helpers (assume lock is already held)
  bool GetKlamptIndices();
  void KlamptToLimb(const Vector& qklampt,int limb,Vector& qlimb) const;
  void LimbToKlampt(const Vector& qlimb,int limb,Vector& qklampt) const;
  void GetKlamptSensedConfig(Vector& qklampt) const;
  void GetKlamptSensedVelocity(Vector& dqklampt) const;
  void GetKlamptCommandedConfig(Vector& qklampt) const;
  void GetKlamptCommandedVelocity(Vector& dqklampt) const;

  //planner interface helpers
  void OnWorldChange();
  void SetLimbGovernor(int limb,int governor);
  bool SolveIK(int limb,const RigidTransform& T,Vector& limbJointPositions,bool doLock);
  bool SolveIKVelocity(int limb,const Vector3& angVel,const Vector3& vel,Vector& limbJointVels,bool doLock);

  ///Subclass should fill this out
  virtual bool MyStartup() { return true; }
  ///Subclass should fill this out
  virtual bool MyProcessSensors() { return true; }
  ///Subclass does not need to fill this out
  virtual void MyAdvanceController();
  ///Subclass should fill this out
  virtual void MySendCommands() {
    if(robotState.leftLimb.sendCommand)
      robotState.leftLimb.sendCommand = false;
    if(robotState.rightLimb.sendCommand)
      robotState.rightLimb.sendCommand = false;
    if(robotState.leftGripper.sendCommand)
      robotState.leftGripper.sendCommand = false;
    if(robotState.rightGripper.sendCommand)
      robotState.rightGripper.sendCommand = false;
    if(robotState.base.sendCommand)
      robotState.base.sendCommand = false;
  }
  ///Subclass should fill this out
  virtual void MyShutdown() {}
  ///Subclass does not need to fill this out
  virtual void MyUpdateSystemStateService();

  Mutex mutex;
  string systemStateAddr;
  bool startup,failure,running,kill;
  RobotState robotState;
  SmartPointer<Robot> robotModel;
  double t,last_t;
  File systemStateService;
  PlannerState planner;

  //klamp't stuff
  vector<int> leftKlamptIndices,rightKlamptIndices;
  int headPanKlamptIndex;
  vector<int> baseKlamptIndices;
  KlamptGripperMap leftGripperMap,rightGripperMap;

  //physical calibration
  BaxterMotorCalibration leftCalibration,rightCalibration;
};

void KlamptGripperMap::CommandToConfig(const double* cmd,double* q) const
{
  for(size_t i=0;i<klamptIndices.size();i++)
    FingerToConfig((int)i,cmd[i],q);
}

void KlamptGripperMap::ConfigToCommand(const double* q,double* cmd) const
{
  for(size_t i=0;i<klamptIndices.size();i++)
    cmd[i] = ConfigToFinger(q,(int)i);
}

void KlamptGripperMap::FingerToConfig(int index,double value,double* q) const
{
  for(size_t i=0;i<klamptIndices[index].size();i++)
    q[klamptIndices[index][i]] = value*klamptScaleCoefs[index][i]+klamptOffsetCoefs[index][i];
}

double KlamptGripperMap::ConfigToFinger(const double* q,int index) const
{
  vector<double> estimates(klamptIndices[index].size());
  for(size_t i=0;i<klamptIndices[index].size();i++)
    estimates[i] = (q[klamptIndices[index][i]]-klamptOffsetCoefs[index][i])/klamptScaleCoefs[index][i];
  double sum = 0;
  for(size_t i=0;i<estimates.size();i++)
    sum += estimates[i];
  return sum / estimates.size();
}

bool ControllerUpdateData::GetKlamptIndices()
{
  bool res = true;
  leftKlamptIndices.resize(numLimbDofs);
  rightKlamptIndices.resize(numLimbDofs);
  for(int i=0;i<numLimbDofs;i++) {
    leftKlamptIndices[i] = robotModel->LinkIndex(klampt_left_limb_names[i]);
    rightKlamptIndices[i] = robotModel->LinkIndex(klampt_right_limb_names[i]);
    if(leftKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain link %s\n",klampt_left_limb_names[i]);
      res = false;
    }
    if(rightKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain link %s\n",klampt_right_limb_names[i]);
      res = false;
    }
    headPanKlamptIndex = robotModel->LinkIndex(klampt_head_name);
    if(headPanKlamptIndex < 0) {
      printf("Klamp't model does not contain link %s\n",klampt_head_name);
      res = false;
    }
  }
  baseKlamptIndices.resize(3);
  for(int i=0;i<3;i++) {
    baseKlamptIndices[i] = robotModel->LinkIndex(klampt_base_names[i]);
    if(baseKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain base link %s\n",klampt_base_names[i]);
      baseKlamptIndices.resize(0);
      res = false;
      break;
    }
  }
  /*
  //DEBUG: see all link names
  for(size_t i=0;i<robotModel->linkNames.size();i++)
    cout<<robotModel->linkNames[i]<<endl;
  */
  int left_gripper_base = robotModel->LinkIndex(klampt_left_gripper_base_name);
  int right_gripper_base = robotModel->LinkIndex(klampt_right_gripper_base_name);
  // printf("Gripper base links: %d %d\n",left_gripper_base,right_gripper_base);
  int left_gripper_num_links = 0;
  int right_gripper_num_links = 0;
  if(left_gripper_base >= 0) {
    if(right_gripper_base >= 0) 
      left_gripper_num_links = right_gripper_base - left_gripper_base;
    else 
      left_gripper_num_links = robotModel->q.n - left_gripper_base;
  }
  if(right_gripper_base >= 0)
    right_gripper_num_links = robotModel->q.n - right_gripper_base;
  if(left_gripper_num_links == 3) {
    //must be rethink electric gripper
    leftGripperMap.name = "Rethink Electric Gripper";
    leftGripperMap.klamptIndices.resize(1);
    leftGripperMap.klamptScaleCoefs.resize(1);
    leftGripperMap.klamptOffsetCoefs.resize(1);
    leftGripperMap.klamptIndices[0].resize(2);
    leftGripperMap.klamptIndices[0][0] = left_gripper_base+1;
    leftGripperMap.klamptIndices[0][1] = left_gripper_base+2;
    leftGripperMap.klamptScaleCoefs[0].resize(2);
    leftGripperMap.klamptScaleCoefs[0][0] = robotModel->qMax[left_gripper_base+1];
    leftGripperMap.klamptScaleCoefs[0][1] = robotModel->qMin[left_gripper_base+2];
    leftGripperMap.klamptOffsetCoefs[0].resize(2,0.0);
  }
  if(left_gripper_num_links == 16 || left_gripper_num_links == 43) {
    //must be reflex gripper
    double proxmax = 2.83;
    double swivelmax = 1.57;
    leftGripperMap.name = "Reflex";
    leftGripperMap.klamptIndices.resize(4);
    leftGripperMap.klamptScaleCoefs.resize(4);
    leftGripperMap.klamptOffsetCoefs.resize(4);
    //finger1
    leftGripperMap.klamptIndices[0].resize(1,robotModel->LinkIndex("left_gripper:proximal_1"));
    leftGripperMap.klamptScaleCoefs[0].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[0].resize(1,proxmax);
    leftGripperMap.klamptIndices[1].resize(1,robotModel->LinkIndex("left_gripper:proximal_2"));
    leftGripperMap.klamptScaleCoefs[1].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[1].resize(1,proxmax);
    leftGripperMap.klamptIndices[2].resize(1,robotModel->LinkIndex("left_gripper:proximal_3"));
    leftGripperMap.klamptScaleCoefs[2].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[2].resize(1,proxmax);
    leftGripperMap.klamptIndices[3].resize(2);
    leftGripperMap.klamptIndices[3][0] = robotModel->LinkIndex("left_gripper:swivel_1");
    leftGripperMap.klamptIndices[3][1] = robotModel->LinkIndex("left_gripper:swivel_2");
    leftGripperMap.klamptScaleCoefs[3].resize(2);
    leftGripperMap.klamptScaleCoefs[3][0] = -swivelmax;
    leftGripperMap.klamptScaleCoefs[3][1] = swivelmax;
    leftGripperMap.klamptOffsetCoefs[3].resize(2,0.0);
  }
  if(right_gripper_num_links == 3) {
    //must be rethink electric gripper
    rightGripperMap.name = "Rethink Electric Gripper";
    rightGripperMap.klamptIndices.resize(1);
    rightGripperMap.klamptScaleCoefs.resize(1);
    rightGripperMap.klamptOffsetCoefs.resize(1);
    rightGripperMap.klamptIndices[0].resize(2);
    rightGripperMap.klamptIndices[0][0] = right_gripper_base+1;
    rightGripperMap.klamptIndices[0][1] = right_gripper_base+2;
    rightGripperMap.klamptScaleCoefs[0].resize(2);
    rightGripperMap.klamptScaleCoefs[0][0] = robotModel->qMax[right_gripper_base+1];
    rightGripperMap.klamptScaleCoefs[0][1] = robotModel->qMin[right_gripper_base+2];
    rightGripperMap.klamptOffsetCoefs[0].resize(2,0.0);
  }
  if(right_gripper_num_links == 16 || right_gripper_num_links == 43) {
    //must be reflex gripper
    double proxmax = 2.83;
    double swivelmax = 1.57;
    rightGripperMap.name = "Reflex";
    rightGripperMap.klamptIndices.resize(4);
    rightGripperMap.klamptScaleCoefs.resize(4);
    rightGripperMap.klamptOffsetCoefs.resize(4);
    //finger1
    rightGripperMap.klamptIndices[0].resize(1,robotModel->LinkIndex("right_gripper:proximal_1"));
    rightGripperMap.klamptScaleCoefs[0].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[0].resize(1,proxmax);
    rightGripperMap.klamptIndices[1].resize(1,robotModel->LinkIndex("right_gripper:proximal_2"));
    rightGripperMap.klamptScaleCoefs[1].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[1].resize(1,proxmax);
    rightGripperMap.klamptIndices[2].resize(1,robotModel->LinkIndex("right_gripper:proximal_3"));
    rightGripperMap.klamptScaleCoefs[2].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[2].resize(1,proxmax);
    rightGripperMap.klamptIndices[3].resize(2);
    rightGripperMap.klamptIndices[3][0] = robotModel->LinkIndex("right_gripper:swivel_1");
    rightGripperMap.klamptIndices[3][1] = robotModel->LinkIndex("right_gripper:swivel_2");
    rightGripperMap.klamptScaleCoefs[3].resize(2);
    rightGripperMap.klamptScaleCoefs[3][0] = -swivelmax;
    rightGripperMap.klamptScaleCoefs[3][1] = swivelmax;
    rightGripperMap.klamptOffsetCoefs[3].resize(2,0.0);
  }
  // printf("Left gripper: %s\n",leftGripperMap.name.c_str());
  // printf("Right gripper: %s\n",rightGripperMap.name.c_str());
  return res;
}

void ControllerUpdateData::KlamptToLimb(const Vector& qklampt,int limb,Vector& qlimb) const
{
  if(limb==LEFT) {
    qlimb.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[leftKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
  }
  else if(limb==RIGHT) {
    qlimb.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[rightKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
  }
  else {
    qlimb.resize(numLimbDofs*2);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[leftKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qlimb[i+numLimbDofs] = qklampt[rightKlamptIndices[i]];
      else
	qlimb[i+numLimbDofs] = 0;
    }
  }
}

void ControllerUpdateData::LimbToKlampt(const Vector& qlimb,int limb,Vector& qklampt) const
{
  if(limb==LEFT) {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qklampt[leftKlamptIndices[i]] = qlimb[i];
    }
  }
  else if(limb==RIGHT) {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qklampt[rightKlamptIndices[i]] = qlimb[i];
    }
  }
  else {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qklampt[leftKlamptIndices[i]] = qlimb[i];
    }
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qklampt[rightKlamptIndices[i]] = qlimb[i+numLimbDofs];
    }
  }
}

void ControllerUpdateData::GetKlamptSensedConfig(Vector& qklampt) const
{
  if(!robotModel) return;
  qklampt = robotModel->q;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.sensedConfig.empty())
      qklampt[leftKlamptIndices[i]] = robotState.leftLimb.sensedConfig[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.sensedConfig.empty())
      qklampt[rightKlamptIndices[i]] = robotState.rightLimb.sensedConfig[i];
  if(robotState.base.enabled) {
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      if(baseKlamptIndices[i] >= 0)
	qklampt[baseKlamptIndices[i]] = robotState.base.odometry[i];
  }
  if(headPanKlamptIndex >= 0)
    qklampt[headPanKlamptIndex] = robotState.head.pan;

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.position.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.position[0],&qklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.position.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.position[0],&qklampt[0]);
}

void ControllerUpdateData::GetKlamptSensedVelocity(Vector& dqklampt) const
{
  if(!robotModel) return;
  dqklampt = robotModel->dq;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.sensedVelocity.empty())
      dqklampt[leftKlamptIndices[i]] = robotState.leftLimb.sensedVelocity[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.sensedVelocity.empty())
      dqklampt[rightKlamptIndices[i]] = robotState.rightLimb.sensedVelocity[i];
  if(robotState.base.enabled && robotState.base.velocity.size() == 3) {
    Vector wvelocity;
    Matrix2 R; R.setRotate(robotState.base.odometry[2]);
    Vector2 wvel = R*Vector2(robotState.base.velocity[0],robotState.base.velocity[1]);
    wvelocity.resize(3);
    wvelocity[0] = wvel.x;
    wvelocity[1] = wvel.y;
    wvelocity[2] = robotState.base.velocity[2];
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      if(baseKlamptIndices[i] >= 0) 
	dqklampt[baseKlamptIndices[i]] = wvelocity[i];
  }
  if(headPanKlamptIndex >= 0) {
    double sign = Sign(robotState.head.panTarget - robotState.head.pan);
    dqklampt[headPanKlamptIndex] = robotState.head.panSpeed*sign;
  }

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.speedCommand.empty()) {
    Vector zero(robotState.leftGripper.speedCommand.n,0.0);
    leftGripperMap.CommandToConfig(&zero[0],&dqklampt[0]);
  }
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.speedCommand.empty()) {
    Vector zero(robotState.rightGripper.speedCommand.n,0.0);
    rightGripperMap.CommandToConfig(&zero[0],&dqklampt[0]);
  }
}

void ControllerUpdateData::GetKlamptCommandedConfig(Vector& qklampt) const
{
  if(!robotModel) return;
  qklampt = robotModel->q;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.commandedConfig.empty())
      qklampt[leftKlamptIndices[i]] = robotState.leftLimb.commandedConfig[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.commandedConfig.empty())
      qklampt[rightKlamptIndices[i]] = robotState.rightLimb.commandedConfig[i];
  if(robotState.base.enabled) {
    if(robotState.base.controlMode == BaseState::RELATIVE_VELOCITY) {
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0) 
	  qklampt[baseKlamptIndices[i]] = robotState.base.odometry[i];
    }
    else {
      Vector tgt;
      robotState.base.GetOdometryTarget(tgt);
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0) 
	  qklampt[baseKlamptIndices[i]] = tgt[i];
    }
  }
  if(headPanKlamptIndex >= 0)
    qklampt[headPanKlamptIndex] = robotState.head.panTarget;

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.positionCommand.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.positionCommand[0],&qklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.positionCommand.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.positionCommand[0],&qklampt[0]);
}


void ControllerUpdateData::GetKlamptCommandedVelocity(Vector& dqklampt) const
{
  if(!robotModel) return;
  dqklampt = robotModel->dq;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.commandedVelocity.empty())
      dqklampt[leftKlamptIndices[i]] = robotState.leftLimb.commandedVelocity[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.commandedVelocity.empty())
      dqklampt[rightKlamptIndices[i]] = robotState.rightLimb.commandedVelocity[i];
  if(robotState.base.enabled) {
    if(robotState.base.controlMode == BaseState::RELATIVE_VELOCITY) {
      Vector wvelocity;
      Matrix2 R; R.setRotate(robotState.base.odometry[2]);
      Vector2 wvel = R*Vector2(robotState.base.command[0],robotState.base.command[1]);
      wvelocity.resize(3);
      wvelocity[0] = wvel.x;
      wvelocity[1] = wvel.y;
      wvelocity[2] = robotState.base.command[2];
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0) 
	  dqklampt[baseKlamptIndices[i]] = wvelocity[i];
    }
    else {
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0) 
	  dqklampt[baseKlamptIndices[i]] = 0;
    }
  }
  if(headPanKlamptIndex >= 0) {
    double sign = Sign(robotState.head.panTarget - robotState.head.pan);
    dqklampt[headPanKlamptIndex] = robotState.head.panSpeed*sign;
  }

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.speedCommand.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.speedCommand[0],&dqklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.speedCommand.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.speedCommand[0],&dqklampt[0]);
}

void LimbState::Advance(Real dt,ControllerUpdateData* controller)
{
  Vector x,v;
  if(motionQueueActive) {
    if(motionQueue.TimeRemaining() >= 0) {
      //update motion queue and set command
      motionQueue.Advance(dt);
      x = motionQueue.CurConfig();
      v = motionQueue.CurVelocity();
    }
  }
  //end effector driving
  if(governor == LimbState::EndEffectorDrive) {
    RigidTransform increment;
    MomentRotation m(driveAngVel * dt);
    m.getMatrix(increment.R);
    increment.t = driveVel * dt;
    RigidTransform desiredTransform;
    desiredTransform.R = increment.R * driveTransform.R;
    desiredTransform.t = increment.t + driveTransform.t;
    //normalize drive transform rotation matrix to correct for numerical error
    //NormalizeRotation(desiredTransform.R);
    Vector qlimb;
    const vector<int>& indices = (limb==LEFT? controller->leftKlamptIndices : controller->rightKlamptIndices);
    Robot* robot = controller->robotModel;
    //cout<<"Driving end effector "<<limb<<" from transform: "<<driveTransform<<endl;
    //cout<<"  dt: "<<dt<<", drive vel "<<driveVel<<", ang vel "<<driveAngVel<<endl;
    //cout<<"  Desired transform: "<<desiredTransform.t<<endl;
    if(controller->SolveIK(limb,desiredTransform,qlimb,false)) {
      //cout<<"  Solved limb position: "<<qlimb<<endl;
      {
	//update achievedTransform
	for(int i=0;i<qlimb.n;i++) 
	  robot->q[indices[i]] = qlimb[i];
	int ee_index = robot->LinkIndex((limb==LEFT?klampt_left_ee_name:klampt_right_ee_name));
	robot->UpdateSelectedFrames(ee_index);
	achievedTransform = robot->links[ee_index].T_World;
	//cout<<"  Solved limb transform: "<<achievedTransform.t<<endl;
      }

      //limit the instantaneous change of movement 
      for(int i=0;i<qlimb.n;i++) {
	if(qlimb[i] - commandedConfig[i] < -robot->velMax[indices[i]]*dt)
	  qlimb[i] = commandedConfig[i] - robot->velMax[indices[i]]*dt;
	if(qlimb[i] - commandedConfig[i] > robot->velMax[indices[i]]*dt)
	  qlimb[i] = commandedConfig[i] + robot->velMax[indices[i]]*dt;
      }

      //update achievedTransform
      for(int i=0;i<qlimb.n;i++) 
	robot->q[indices[i]] = qlimb[i];
      int ee_index = robot->LinkIndex((limb==LEFT?klampt_left_ee_name:klampt_right_ee_name));
      robot->UpdateSelectedFrames(ee_index);
      achievedTransform = robot->links[ee_index].T_World;
      //cout<<"  Solved limb transform, limited to max velocity: "<<achievedTransform.t<<endl;
      
      //adjust drive transform along screw to minimize distance to the achieved transform      
      Vector3 trel = achievedTransform.t - driveTransform.t;
      Matrix3 Rrel;
      Rrel.mulTransposeB(achievedTransform.R,driveTransform.R);
      Vector3 axis = driveVel / Max(driveVel.length(),Epsilon);
      Vector3 rotaxis = driveAngVel / Max(driveAngVel.length(),Epsilon);
      Real tdistance = trel.dot(axis);
      //cout<<"  translation vector"<<trel<<endl;
      //printf("  Translation amount: %g\n",tdistance);
      tdistance = Clamp(tdistance,0.0,dt);
      Real Rdistance = AxisRotationMagnitude(Rrel,driveAngVel);
      //printf("  Rotation amount: %g\n",Rdistance);
      Rdistance = Clamp(Rdistance,0.0,dt);
      Real ut=0,uR=0;
      ut = driveVel.length();
      uR = driveAngVel.length();
      Real distance = (tdistance*ut+Rdistance*uR)/Max(ut+uR,Epsilon);
      //printf("  Drive amount: %g\n",distance);
      //computed error-minimizing distance along screw motion
      driveTransform.t.madd(axis,distance);
      m.set(rotaxis*distance);
      m.getMatrix(increment.R);
      driveTransform.R = increment.R * driveTransform.R;
      //NormalizeRotation(driveTransform.R);

      x = qlimb;
      v = (qlimb - commandedConfig) * (1.0/dt);
      sendCommand = true;
    }
    else {
      //don't adjust drive transform
      printf("  IK solve failed\n");
    }
  }
  //copy joint command to limb state
  if(!x.empty()) {
    if(enableSelfCollisionAvoidance)
      controlMode = LimbState::POSITION;
    else
      controlMode = LimbState::RAW_POSITION;
    sendCommand = true;
    commandedConfig = x;
    commandedVelocity = v;
    rawCommandToSend = x;
  }
  if(controlMode == LimbState::VELOCITY) {
    //integrate commanded config
    commandedConfig.madd(commandedVelocity,dt);
    rawCommandToSend = commandedVelocity;
  }
  if(controlMode == LimbState::EFFORT)
    rawCommandToSend = commandedEffort;

  //TODO: difference commanded velocity to get commanded velocity?

  //test whether to use the motor calibration
  BaxterMotorCalibration* calib = NULL;
  if(limb == LEFT)
    calib = &controller->leftCalibration;
  else if(limb == RIGHT) 
    calib = &controller->rightCalibration;

  if(integralError.empty())
    integralError.resize(sensedConfig.size(),0.0);
  for(int i=0;i<integralError.n;i++) {
    //inverse weight the error by the velocity to avoid the "catch up"
    //overshoot
    double ei = commandedConfig[i] - sensedConfig[i];
    ei /= 10.0*Max(0.1,Abs(commandedVelocity[i]),Abs(sensedVelocity[i]));
    integralError[i] = Clamp(integralError[i]+ei*dt,-0.2,0.2);
  }

  if(calib && calib->Enabled() && !commandedConfig.empty() && (controlMode == LimbState::POSITION || controlMode == LimbState::RAW_POSITION)) {
    //TODO: MPC-based control
    vector<Vector> qdesired(1,commandedConfig);
    Config qsense_extrap = sensedConfig;
    if(senseUpdateTime >= 0)  {
      //cout<<"Sensed: "<<sensedConfig<<endl;
      //printf("Extrapolating velocity by %g seconds\n",controller->t-senseUpdateTime);
      qsense_extrap.madd(sensedVelocity,controller->t-senseUpdateTime);
    }
    else
      printf("Unable to extrapolate velocity\n");
    x.resize(commandedConfig.size());
    //compute gravity vector g
    Robot* robot = controller->robotModel;
    const vector<int>& indices = (limb==LEFT? controller->leftKlamptIndices : controller->rightKlamptIndices);
    for(size_t i=0;i<indices.size();i++) 
      robot->q[indices[i]] = commandedConfig[i];
    int ee_index = robot->LinkIndex((limb==LEFT?klampt_left_ee_name:klampt_right_ee_name));
    robot->UpdateSelectedFrames(ee_index);
    Vector G;
    robot->GetGravityTorques(Vector3(0,0,-9.8),G);
    Vector g((int)indices.size());
    for(size_t i=0;i<indices.size();i++) 
      g[i] = G[indices[i]];
    
    //compute calibrated command
    calib->ToCommand(qsense_extrap,sensedVelocity,g,qdesired,x);

    //set the raw command
    double settleTime = 2.0;
    double integralGain = 0.5;
    if(controller->t < settleTime) {
      double fadein = controller->t / settleTime;
      rawCommandToSend.madd(x-rawCommandToSend, fadein);
      rawCommandToSend.madd(integralError,integralGain*fadein);
    }
    else {
      rawCommandToSend = x;
      rawCommandToSend.madd(integralError,integralGain);
    }
    for(int i=0;i<rawCommandToSend.n;i++)
      rawCommandToSend[i] = Clamp(rawCommandToSend[i],robot->qMin(indices[i]),robot->qMax(indices[i]));
    sendCommand = true;
  }
}

void ControllerUpdateData::MyAdvanceController()
{
  ScopedLock lock(mutex);
  if(planner.plannerActive[0] || planner.plannerActive[1]) {
    if(planner.planningThread.SendUpdate(planner.motionQueueInterface)) {
      //copy the update to the limbs' motion queues
      if(planner.plannerActive[LEFT]) {
	robotState.leftLimb.motionQueue.pathOffset = planner.wholeRobotMotionQueue.pathOffset;
	for(size_t i=0;i<leftKlamptIndices.size();i++)
	  robotState.leftLimb.motionQueue.path.elements[i] = planner.wholeRobotMotionQueue.path.elements[leftKlamptIndices[i]];
      }
      if(planner.plannerActive[RIGHT]) {
	robotState.rightLimb.motionQueue.pathOffset = planner.wholeRobotMotionQueue.pathOffset;
	for(size_t i=0;i<rightKlamptIndices.size();i++)
	  robotState.rightLimb.motionQueue.path.elements[i] = planner.wholeRobotMotionQueue.path.elements[rightKlamptIndices[i]];
      }
    }
  }
  double dt = t-last_t;
  robotState.leftLimb.Advance(dt,this);
  robotState.rightLimb.Advance(dt,this);
}

void ControllerUpdateData::MyUpdateSystemStateService()
{
  if(!systemStateService.IsOpen()) return;

#if USE_SSPP
  //provide feedback to the state service about the time
  {
    AnyCollection msg;
    {
      ScopedLock lock(mutex);
      msg["t"] = t;
      msg["dt"] = t-last_t;
    }
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".robot");
    command["data"] = msg;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  //provide feedback about the command
  {
    Vector q,dq;
    Vector larmq,larmv,rarmq,rarmv,baseq,basev;
    double hpan,hpanspeed;
    {
      ScopedLock lock(mutex);
      larmq = robotState.leftLimb.commandedConfig;
      larmv = robotState.leftLimb.commandedVelocity;
      rarmq = robotState.rightLimb.commandedConfig;
      rarmv = robotState.rightLimb.commandedVelocity;
      hpan = robotState.head.panTarget;
      hpanspeed = robotState.head.panSpeed;
      baseq.resize(3,0.0);
      if(robotState.base.enabled && robotState.base.controlMode != BaseState::NONE) {
	if(!robotState.base.GetOdometryTarget(baseq))
	  basev = robotState.base.command;
      }
      GetKlamptCommandedConfig(q);
      GetKlamptCommandedVelocity(dq);
    }
    //TODO: gripper feedback
    AnyCollection commandData;   
    if(!q.empty()) {
      commandData["q"] = vector<double>(q);
      commandData["dq"] = vector<double>(dq);
    }
    commandData["left"] = AnyCollection();
    commandData["left"]["q"] = vector<double>(larmq);
    commandData["left"]["dq"] = vector<double>(larmv);
    commandData["right"] = AnyCollection();
    commandData["right"]["q"] = vector<double>(rarmq);
    commandData["right"]["dq"] = vector<double>(rarmv);
    commandData["head"] = AnyCollection();
    commandData["head"]["pan"] = hpan;
    commandData["head"]["panspeed"] = hpanspeed;
    if(!baseq.empty() || !basev.empty()) {
      commandData["base"]["odometry"] = vector<double>(baseq);
      commandData["base"]["velocity"] = vector<double>(basev);
    }
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.command");
    command["data"] = commandData;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }


  //provide feedback about the sensed
  {
    //TODO: gripper feedback
    Vector q,dq;
    Vector larmq,larmv,rarmq,rarmv,baseq,basev;
    double hpan;
    int hnod;
    {
      ScopedLock lock(mutex);
      larmq = robotState.leftLimb.sensedConfig;
      larmv = robotState.leftLimb.sensedVelocity;
      rarmq = robotState.rightLimb.sensedConfig;
      rarmv = robotState.rightLimb.sensedVelocity;
      hpan = robotState.head.pan;
      hnod = robotState.head.isNodding;
      baseq.resize(3,0.0);
      if(robotState.base.enabled) {
	baseq = robotState.base.odometry;
	basev = robotState.base.velocity;
      }
      GetKlamptSensedConfig(q);
      GetKlamptSensedVelocity(dq);
    }
    AnyCollection sensedData;   
    if(!q.empty()) {
      sensedData["q"] = vector<double>(q);
      sensedData["dq"] = vector<double>(dq);
    }
    sensedData["left"]["q"] = vector<double>(larmq);
    sensedData["left"]["dq"] = vector<double>(larmv);
    sensedData["right"]["q"] = vector<double>(rarmq);
    sensedData["right"]["dq"] = vector<double>(rarmv);
    sensedData["head"]["pan"] = hpan;
    sensedData["head"]["nodding"] = hnod;
    if(!baseq.empty()) {
      sensedData["base"]["odometry"] = vector<double>(baseq);
      sensedData["base"]["velocity"] = vector<double>(basev);
    }
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.sensed");
    command["data"] = sensedData;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  if (robotState.leftLimb.motionQueueActive) {
    PolynomialMotionQueue* queue = &robotState.leftLimb.motionQueue;
    //provide feedback about the motion queue
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.left");
    AnyCollection data;
    double tstart = queue->CurTime();
    double tend = queue->TimeRemaining()+tstart;
    data["traj_t_end"] = tend;
    //read out the path?
    Config q;
    Real dt = 0.05;
    int istart=(int)Ceil(tstart/dt);
    int iend=(int)Ceil(tend/dt);
    AnyCollection path;
    vector<double> times;
    for(int i=istart;i<iend;i++) {
      Real t=i*dt;
      times.push_back(t);
      queue->Eval(t,q,false);
      path[i-istart] = vector<double>(q);
    }
    q = queue->Endpoint();
    path[iend-istart] = vector<double>(q);
    times.push_back(tend);
    /*
      data["traj"]["milestones"] = path;
      data["traj"]["times"] = times;
    */
    data["traj_q_end"] = vector<double>(q);
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  else {
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.left");
    command["data"] = AnyCollection();
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  if (robotState.rightLimb.motionQueueActive) {
    PolynomialMotionQueue* queue = &robotState.rightLimb.motionQueue;
    //provide feedback about the motion queue
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.right");
    AnyCollection data;
    double tstart = queue->CurTime();
    double tend = queue->TimeRemaining()+tstart;
    data["traj_t_end"] = tend;
    //read out the path?
    Config q;
    Real dt = 0.05;
    int istart=(int)Ceil(tstart/dt);
    int iend=(int)Ceil(tend/dt);
    AnyCollection path;
    vector<double> times;
    for(int i=istart;i<iend;i++) {
      Real t=i*dt;
      times.push_back(t);
      queue->Eval(t,q,false);
      path[i-istart] = vector<double>(q);
    }
    q = queue->Endpoint();
    path[iend-istart] = vector<double>(q);
    times.push_back(tend);
    /*
      data["traj"]["milestones"] = path;
      data["traj"]["times"] = times;
    */
    data["traj_q_end"] = vector<double>(q);
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  else {
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.right");
    command["data"] = AnyCollection();
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }


  //provide feedback about the end effectors
  if(robotModel != NULL) {
    int ee[2] = {robotModel->LinkIndex(klampt_left_ee_name),robotModel->LinkIndex(klampt_right_ee_name)};
    AnyCollection data;
    data.resize(2);
    GetKlamptSensedConfig(robotModel->q);
    robotModel->UpdateFrames();
    for(int i=0;i<2;i++) {
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[i]].T_World.R.get(&R[0]);
      robotModel->links[ee[i]].T_World.t.get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[i]["xform"]["sensed"] = xform;
    }
    GetKlamptCommandedConfig(robotModel->q);
    robotModel->UpdateFrames();
    for(int i=0;i<2;i++) {
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[i]].T_World.R.get(&R[0]);
      robotModel->links[ee[i]].T_World.t.get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[i]["xform"]["commanded"] = xform;
    }
    //TODO: trajectory destination
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.endEffectors");
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
#endif // USE_SSPP
}


bool BaseState::GetRelativeTarget(Vector& trel) const
{
  if(controlMode == RELATIVE_VELOCITY)
    //relative velocity mode
    return 0;
  else if(controlMode == RELATIVE_POSITION) {
    trel = command;
  }
  else {
    //do transformation to local coordinates
    ToLocal(command,trel);
  }
  return 1;
}

bool BaseState::GetOdometryTarget(Vector& odo) const
{
  if(controlMode == RELATIVE_VELOCITY)
    //relative velocity mode
    return 0;
  else if(controlMode == RELATIVE_POSITION) {
    //do transformation from local to odometry coordinates
    ToOdometry(command,odo);
  }
  else {
    odo = command;
  }
  return 1;
}


void BaseState::ToLocal(const Vector& odoconfig,Vector& lconfig) const
{
  double xo=odometry[0];
  double yo=odometry[1];
  double thetao=odometry[2];
  double xabs=odoconfig[0];
  double yabs=odoconfig[1];
  double thetaabs=odoconfig[2];
  Vector2 dpabs (xabs-xo,yabs-yo);
  lconfig.resize(3);
  lconfig[2] = AngleDiff(thetaabs,thetao);
  Matrix2 rrel; rrel.setRotate(-thetao);
  Vector2 dprel = rrel*dpabs;
  dprel.get(lconfig[0],lconfig[1]);
}

void BaseState::ToOdometry(const Vector& lconfig,Vector& odoconfig) const
{
  double xo=odometry[0];
  double yo=odometry[1];
  double thetao=odometry[2];
  double xrel=lconfig[0];
  double yrel=lconfig[1];
  double thetarel=lconfig[2];
  Vector2 dprel (xrel,yrel);
  Matrix2 R; R.setRotate(thetao);
  Vector2 dpabs = R*dprel;
  odoconfig.resize(3);
  odoconfig[0] = dpabs.x;
  odoconfig[1] = dpabs.y;
  odoconfig[2] = thetao+thetarel;
}


void ControllerUpdateData::SetLimbGovernor(int limb,int governor)
{
  if((limb == LEFT && robotState.leftLimb.governor == governor) ||
     (limb == RIGHT && robotState.rightLimb.governor == governor)) 
    return;
  if(limb == BOTH && robotState.leftLimb.governor == governor && robotState.rightLimb.governor == governor) 
    return;

  //activate motion queue, if not already
  if(limb == LEFT || limb == BOTH) {
    //first, determine whether to activate or deactivate motion queue
    if(governor == LimbState::MotionQueue || governor == LimbState::Planner) {
      if(!robotState.leftLimb.motionQueueActive) {
	robotState.leftLimb.motionQueue.SetConstant(robotState.leftLimb.commandedConfig);
	if(robotModel) {
	  KlamptToLimb(robotModel->qMin,LEFT,robotState.leftLimb.motionQueue.qMin);
	  KlamptToLimb(robotModel->qMax,LEFT,robotState.leftLimb.motionQueue.qMax);
	  KlamptToLimb(robotModel->velMax,LEFT,robotState.leftLimb.motionQueue.velMax);
	  KlamptToLimb(robotModel->accMax,LEFT,robotState.leftLimb.motionQueue.accMax);
	}
      }
      robotState.leftLimb.motionQueueActive = true;
    }
    else
      robotState.leftLimb.motionQueueActive = false;

    if(governor == LimbState::EndEffectorDrive) {
      //begin drive mode
      double R[9];
      Vector3 t;
      GetKlamptCommandedConfig(robotModel->q);
      //ignore base
      for(size_t i=0;i<baseKlamptIndices.size();i++) {
	robotModel->q[baseKlamptIndices[i]] = 0;
      }
      int index=robotModel->LinkIndex(klampt_left_ee_name);
      robotModel->UpdateSelectedFrames(index);
      RigidTransform Tcur = robotModel->links[index].T_World;
      robotState.leftLimb.driveTransform = Tcur;
      robotState.leftLimb.achievedTransform = Tcur;
    }

    robotState.leftLimb.governor = governor;
  }
  if(limb == RIGHT || limb == BOTH) {
    //first, determine whether to activate or deactivate motion queue
    if(governor == LimbState::MotionQueue || governor == LimbState::Planner) {
      if(!robotState.rightLimb.motionQueueActive) {
	robotState.rightLimb.motionQueue.SetConstant(robotState.rightLimb.commandedConfig);
	if(robotModel) {
	  KlamptToLimb(robotModel->qMin,RIGHT,robotState.rightLimb.motionQueue.qMin);
	  KlamptToLimb(robotModel->qMax,RIGHT,robotState.rightLimb.motionQueue.qMax);
	  KlamptToLimb(robotModel->velMax,RIGHT,robotState.rightLimb.motionQueue.velMax);
	  KlamptToLimb(robotModel->accMax,RIGHT,robotState.rightLimb.motionQueue.accMax);
	}
      }
      robotState.rightLimb.motionQueueActive = true;
    }
    else {  
      robotState.rightLimb.motionQueueActive = false;
    }

    if(governor == LimbState::EndEffectorDrive) {
      //begin drive mode
      double R[9];
      Vector3 t;
      GetKlamptCommandedConfig(robotModel->q);
      //ignore base
      for(size_t i=0;i<baseKlamptIndices.size();i++) {
	robotModel->q[baseKlamptIndices[i]] = 0;
      }
      int index=robotModel->LinkIndex(klampt_right_ee_name);
      robotModel->UpdateSelectedFrames(index);
      RigidTransform Tcur = robotModel->links[index].T_World;
      robotState.rightLimb.driveTransform = Tcur;
      robotState.rightLimb.achievedTransform = Tcur;
    }

    robotState.rightLimb.governor = governor;
  }

  if(governor == LimbState::Planner) {
    if(!planner.plannerActive[0]) {
      planner.plannerActive[0] = true;
      planner.plannerActive[1] = true;
      Vector qstart;
      GetKlamptCommandedConfig(qstart);
      planner.planningThread.SetStartConfig(qstart);
      planner.planningThread.SetPlanner(new RealTimePlanner);
      planner.planningThread.SetCSpace(planner.cspace);
      planner.planningThread.Start();
    }
  }
  else {
    if(planner.plannerActive[0] && planner.plannerActive[1]) {
      planner.plannerActive[0] = false;
      planner.plannerActive[1] = false;
      planner.planningThread.Stop();
    }
  }
}

bool ControllerUpdateData::SolveIK(int limb,const RigidTransform& T,Vector& limbJointPositions,bool doLock)
{
  int index;
  const vector<int>* limbIndices = NULL;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
  }
  else 
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  if(doLock) mutex.lock();
  GetKlamptCommandedConfig(robotModel->q);
  GetKlamptCommandedVelocity(robotModel->dq);
  robotModel->UpdateFrames();
  IKGoal goal;
  goal.link = index;
  goal.SetFixedRotation(T.R);
  goal.SetFixedPosition(T.t);
  RobotIKFunction function(*robotModel);
  function.UseIK(goal);
  function.activeDofs.mapping = *limbIndices;
  Real tolerance = 1e-5;
  int iters = 100;
  int verbose = 0;
  bool res = ::SolveIK(function,tolerance,iters,verbose);
  KlamptToLimb(robotModel->q,limb,limbJointPositions);
  if(doLock) mutex.unlock();
  return res;
}

bool ControllerUpdateData::SolveIKVelocity(int limb,const Vector3& angVel,const Vector3& vel,Vector& limbJointVels,bool doLock)
{
  int index;
  const vector<int>* limbIndices = NULL;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
  }
  else 
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  if(doLock) mutex.lock();
  GetKlamptCommandedConfig(robotModel->q);
  GetKlamptCommandedVelocity(robotModel->dq);
  robotModel->UpdateSelectedFrames(index);
  robotModel->GetFullJacobian(Vector3(0.0),index,J);
  if(doLock) mutex.unlock();

  Jlimb.resize(6,limbIndices->size());
  for(int i=0;i<limbIndices->size();i++)
    Jlimb.copyCol(i,J.col((*limbIndices)[i]));
  Vector twist(6);
  twist[0] = angVel[0];
  twist[1] = angVel[1];
  twist[2] = angVel[2];
  twist[3] = vel[0];
  twist[4] = vel[1];
  twist[5] = vel[2];
  RobustSVD<Real> svd;
  if(!svd.set(Jlimb)) {
    printf("sendEndEffectorVelocity: failed to solve for Jacobian pseudoinverse\n");
    return false;
  }
  svd.dampedBackSub(twist,1e-3,limbJointVels);
  //limit to joint velocities
  Real scale = 1.0;
  for(int i=0;i<limbJointVels.n;i++) {
    int k=(*limbIndices)[i];
    if(limbJointVels[i]*scale < robotModel->velMin[k])
      scale = robotModel->velMin[k]/limbJointVels[i];
    if(limbJointVels[i]*scale > robotModel->velMax[k])
      scale = robotModel->velMax[k]/limbJointVels[i];
  }
  limbJointVels *= scale;
  return true;
}

void ControllerUpdateData::OnWorldChange()
{
  if(planner.world.robots.empty()) {
    if(robotModel) {
      planner.world.robots.resize(1);
      planner.world.robots[0].name = "Apc Robot";
      planner.world.robots[0].robot = robotModel;
      planner.world.robots[0].view.robot = robotModel;
      planner.world.robots[0].view.SetGrey();
    }
  }
  else {
    if(robotModel) {
      printf("Warning, world file has a robot in it... replacing robot 0 with previously loaded model\n");
      planner.world.robots[0].robot = robotModel;
      planner.world.robots[0].view.robot = robotModel;
      planner.world.robots[0].view.SetGrey();
    }
    else {
      printf("Warning, world file has a robot in it... is it the same as the Klamp't model?\n");
      robotModel = planner.world.robots[0].robot;
    }
  }
  planner.settings.InitializeDefault(planner.world);
  planner.cspace = new SingleRobotCSpace(planner.world,0,&planner.settings);
  planner.planningThread.SetCSpace(planner.cspace);
}





BaxterMotorCalibration::BaxterMotorCalibration()
: lookahead(1), rate(50)
{}

bool BaxterMotorCalibration::Enabled() const
{
  return !Brows.empty();
}

void BaxterMotorCalibration::Clear()
{
  Brows.clear();
  Crows.clear();
  d.clear();
}

bool BaxterMotorCalibration::Load(const char* fn,const char* limb)
{
  name = limb;
  AnyCollection c;
  ifstream in(fn,ios::in);
  if(!in) {
    fprintf(stderr,"Unable to open calibration file %s\n",fn);
    return false;
  }
  in >> c;
  if(!in) {
    fprintf(stderr,"Unable to parse calibration from %s\n",fn);
    return false;
  }
  if(c.find(limb) == NULL) {
    fprintf(stderr,"Unable to load calibration for limb from %s\n",limb,fn);
    return false;
  }
  AnyCollection climb = c[limb];
  AnyCollection B = climb["B"];
  AnyCollection C = climb["C"];
  AnyCollection d = climb["d"];
  Assert(B.size()==7);
  Assert(C.size()==7);
  Assert(d.size()==7);
  Brows.resize(B.size());
  Assert(B.isarray());
  Assert(C.isarray());
  Assert(d.isarray());
  Crows.resize(C.size());
  this->d.resize(d.size());
  for(int i=0;i<7;i++) {
    Assert(B[i].ismap());
    Assert(C[i].ismap());
    Brows[i].clear();
    Crows[i].clear();
    vector<int> keys;
    vector<Real> values;
    bool res=B[i]["columns"].asvector<int>(keys);
    if(!res) {
      fprintf(stderr,"Invalid key in B matrix\n",fn);
      return false;
    }
    res = B[i]["values"].asvector<Real>(values);
    if(!res) {
      fprintf(stderr,"Invalid value in B matrix\n",fn);
      return false;
    }
    Assert(keys.size()==values.size());
    //printf("Loaded %d elements from B matrix entry %d\n",keys.size(),i);
    for(size_t j=0;j<keys.size();j++) {
      Brows[i][keys[j]] = values[j];
    }
    keys.resize(0);
    values.resize(0);
    res=C[i]["columns"].asvector<int>(keys);
    if(!res) {
      fprintf(stderr,"Invalid key in C matrix\n",fn);
      return false;
    }
    res = C[i]["values"].asvector<Real>(values);
    if(!res) {
      fprintf(stderr,"Invalid value in C matrix\n",fn);
      return false;
    }
    Assert(keys.size()==values.size());
    //printf("Loaded %d elements from C matrix entry %d\n",keys.size(),i);
    for(size_t j=0;j<keys.size();j++) {
      Crows[i][keys[j]] = values[j];
    }
    this->d[i] = Real(d[i]);
  }
  return true;
}

void BaxterMotorCalibration::ToCommand(const Vector& q,const Vector& v,const Vector& g,const vector<Vector>& qdesired,Vector& qcmd)
{
  Assert(qdesired.size() > 0);
  if(!Enabled()) {
    printf("Calibration not enabled\n");
    qcmd = qdesired[0];
    return;
  }
  qcmd.resize(q.n);
  qcmd.set(0.0);
  for(size_t i=0;i<Brows.size();i++) {
    if(Brows[i].numEntries()==0) {
      qcmd[i] = qdesired[0][i];
    }
    for(SparseVector::iterator e=Brows[i].begin();e!=Brows[i].end();e++) {
      int index = e->first;
      int future = index / q.size();
      int joint = index % q.size();
      if(future >= (int)qdesired.size())
	future = (int)qdesired.size()-1;
      qcmd[i] += qdesired[future][joint]*e->second;
    }
    for(SparseVector::iterator e=Crows[i].begin();e!=Crows[i].end();e++) {
      int index = e->first;
      int item = index / q.size();
      int joint = index % q.size();
      if(item == 0)
	qcmd[i] += q[joint]*e->second;
      else if(item == 1)
	qcmd[i] += v[joint]*e->second;
      else if(item == 2) {
	if(!g.empty()) 
	  qcmd[i] += g[joint]*e->second;
      }
      else {
	printf("Warning: calibration refers to an invalid state feature\n");
      }
    }      
  }
  qcmd += d;
  /*
  cout<<name<<endl;
  cout<<"Q: "<<q<<endl;
  cout<<"V: "<<v<<endl;
  cout<<"G: "<<g<<endl;
  cout<<"Desired: "<<qdesired[0]<<endl;
  cout<<"Output: "<<qcmd<<endl;
  */
}
