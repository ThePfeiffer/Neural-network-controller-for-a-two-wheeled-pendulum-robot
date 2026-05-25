 /***************************************************************************
 * definition file for the regbot.
 * The main function is the controlTick, that is called 
 * at a constant time interval (of probably 1 ms, see the intervalT variable)
 * 
 * The main control functions are in control.cpp file
 * 
 * ***************************************************************************
 *   Copyright (C) 2019-2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

 /*
  * nov 2024, moved balance to before acceleration control to allow
  *           limited acceleration when starting balance. implemented limited-flag
  *           for all integrators in the balance chain,
  *           didn't seem to help as much as expected. */
 
#include "ucontrol.h"
#include "ulog.h"
// #include "motor_controller.h"
//#include "serial_com.h" 
// #include "robot.h"
#include "umission.h"
#include "ulinesensor.h"
// #include "dist_sensor.h"
#include "ueeconfig.h"
#include "uservo.h"
// #include "mpu9150.h"

#include "uusb.h"
#include "umotor.h"
#include "umission.h"
#include "urobot.h"
#include "uencoder.h"
#include "uirdist.h"
#include "uimu2.h"
#include "udisplay.h"
#include "uNN.h"

UControl control;


// int mission = 0;
int missionButtonCount = 0;
const int missionMax =4;
const char * missionName[missionMax] = // NB! NO spaces in name"
                            { "User_mission",
                              "Balance_on_spot",
                              "Balance_square",
                              "Follow_wall"
                            };
UMissionLine * misLine = NULL;  // current mission line    "User_mission"};
UMissionLine * misLineLast = NULL;  // last mission line    "User_mission"};
// uint8_t misThread = 0;  // current thread number (setwhen mis-line is set)
int8_t missionLineNum = 0;         // current mission line number
float misStartDist;         // start distance for this line
int misLast = -1;
int misStateLast = -1;
  

//////////////////////////////////////  
//////////////////////////////////////  
//////////////////////////////////////  

void UControl::setup()
{
  usb.addSubscriptionService(this);
  // add all the controllers
  ctrlVelLeft = new UControlBase("cvel", "Wheel velocity control");
  ctrlVelRight = new UControlBase("cvel", "Wheel velocity control"); // same ID (always equal parameters)
  ctrlTurn = new UControlTurn("ctrn", "Heading control");
  ctrlWallVel = new UControlIrVel("cwve", "Front distance control");
  ctrlWallTurn  = new UControlWallTurn("cwth", "Wall distance control");
  ctrlPos  = new UControlPos("cpos", "Drive distance control");
  ctrlEdge  = new UControlEdge("cedg", "Edge follow control");
  ctrlBal = new UControlBase("cbal", "Balance angle control");
  ctrlBalVel = new UControlBalVel("cbav", "Balance velocity control");
  ctrlBalPos = new UControlPos("cbap", "Balance drive position control");
  // factory reset left (will be overwritten, if configuration is saved to flash)
  ctrlVelLeft->ffUse = true;
  ctrlVelLeft->ffKp = 1;
  ctrlVelLeft->useCtrl = true;
  ctrlVelLeft->kp = 0;
  ctrlVelLeft->outLimitUse = true;
  ctrlVelLeft->outLimit = 8.0;
  // factory reset right
  ctrlVelRight->ffUse = true;
  ctrlVelRight->ffKp = 1;
  ctrlVelRight->useCtrl = true;
  ctrlVelRight->kp = 0;
  ctrlVelRight->outLimitUse = true;
  ctrlVelRight->outLimit = 8.0;
  // init other control units - default is not-use
  // set mission flags
  controlActive = false;
  missionStateLast = -1;
  mission = 0;
  missionState = 0;
  rateLimitUse = false;
  rateLimitUsed = false;
  // connect regulator in and out
  setRegulatorInOut();
  resetControl();
  versioncpp = "$Id: ucontrol.cpp 1746 2025-12-30 17:02:28Z jcan $";
}

bool UControl::decode(const char* line)
{
  bool used = true;
  if (strncmp(line, "rc ", 3) == 0)
  { // remote control
    const char * p1 = &line[3];
    int cs = strtol(p1, (char**)&p1, 10);
    float lv = strtof(p1, (char**)&p1);
    float hv = strtof(p1, (char**)&p1);
    int bal = strtol(p1, (char**)&p1, 10);
    setRemoteControl(cs, lv, hv, bal);
  }
  else if (strncmp(line, "mh ", 3) == 0)
  { // starting a hard-coded mission
    const char * p1 = &line[3];
    int mh = strtol(p1, (char**)&p1, 10);
    if (mh < eeConfig.hardConfigCnt)
    { // works if controlActive=false, ie no active mission or RC
      mission = mh;
    }
  }
  else
  {
    used = false;
  }
  return used;
}


void UControl::sendHelp()
{
  usb.send("# Control settings -------\r\n");
  usb.send("# -- \trc P lv hv [b]\tRemote control P=0: off, P=1:manual, P>1: mission; lv=linear vel (m/s); hv = heading vel (ish); b=1 for balance\r\n");
  usb.send("# -- \tmh n \tHard coded mission: n=1 balance 5s, n=2 square, n=3 ?, n=?\r\n");
  usb.send("# -- \tparams: use kp iuse itau ilim Lfuse LfNum LfDen Lbuse LbNum LbDen preUse preNum preDen\r\n");
  usb.send("# -- \t        preIuse preItau preIlim ffUse ffKp ffFuse ffFnum ffFden LimUse Lim\r\n");
}


void UControl::stopSubscriptions()
{
  ctrlVelLeft->stopSubscriptions();
  ctrlTurn->stopSubscriptions();
  ctrlWallVel->stopSubscriptions();
  ctrlWallTurn->stopSubscriptions();
  ctrlPos->stopSubscriptions();
  ctrlEdge->stopSubscriptions();
  ctrlBal->stopSubscriptions();
  ctrlBalVel->stopSubscriptions();
  ctrlBalPos->stopSubscriptions();
}



void UControl::setRemoteControl(int ctrlState, float vel, float velDif, bool balance)
{
  controlState = ctrlState;
  if (remoteControl and not ctrlState and missionState > 0)
  { // turning off remote control - need to reset some parts
    mission_turn_ref = encoder.pose[2];
    // acceleration limitation must start from current velocity
    vel_ref_pre_bal[0] = encoder.wheelVelocityEst[0];
    vel_ref_pre_bal[1] = encoder.wheelVelocityEst[1];
    balance_active = savedBalanceActive;
    controlActive = savedControlActive;
//     if (missionState == 0)
//     { // no mission in progress, so disable motors
//       controlActive = false;
//       motorSetEnable(false, false);
//       servo.releaseServos();
//     }
//     usb.send("# UControl::setRemoteControl back to mission\n");
  }
  else if (ctrlState> 0 and ctrlState < 10 and not remoteControl)
  { // turn on remote control
    savedBalanceActive = controlActive and balance_active;
    savedControlActive = controlActive;
    // start servo steering
    // servo.servo1isSteering = true;
//     servo.setServoSteering();
    // make sure control is active
    controlActive = true;
    motor.motorSetEnable(true, true);
//     usb.send("# UControl::setRemoteControl to RC\n");
  }
  remoteControl = ctrlState > 0 and ctrlState < 10;
  remoteControlVel = vel;
  remoteControlVelDif = velDif;
  remoteControlNewValue = true;
  balance_active = balance;
  // debug
  if (false)
  {
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# UControl::setRemoteControl: controlState=%d, rc=%d\n", controlState, remoteControl);
    usb.send(s);
  }
  // debug end
}

void UControl::sendMissionStatusChanged(bool forced)
{
  const int MSL = 120;
  char s[MSL];
  if (forced or mission != misLast or missionState != misStateLast or 
     (misLine != NULL and (misLine != misLineLast or misThread != misThreadLast)))
  {
    // mission thread and line number is of the latest implemented line and thread
    snprintf(s, MSL, "mis %d %d %d '%s' %d %d\r\n",
            mission, missionState , missionLineNum, 
            missionName[mission],
            0, // unused (was sendStatusWhileRunning)
            misThread
          );
    usb.send(s);
    misLast = mission;
    misStateLast = missionState;
    misLineLast = misLine;
    misThreadLast = misThread;
  }
  if (false and misLine != NULL and (misLine != misLineLast or misThread != misThreadLast))
  {
    char * p1;
    int n;
    snprintf(s, MSL, "#mis %d %d %d %d'",
             mission, missionState, missionLineNum, misThread);
    n = strlen(s);
    p1 = &s[n];
    n += misLine->toString(p1, MSL - n - 4, false);
    p1 = &s[n];
    strncpy(p1, "'\r\n", 4);
    usb.send(s);
    misLineLast = misLine;
    misThreadLast = misThread;
  }
}

/**
 * Step on position, as specified by GUI
 * parameter estimate mission (should use fast logging) */
bool UControl::mission_hard_coded()
{
//   const int MSL = 60;
//   char s[MSL];
  bool isOK = false;
  switch (missionState)
  {
    case 0: // wait for start button
      if (robot.button or robot.missionStart)
      { // goto next state
        userMission.missionTime = 0.0;
        missionState = 1;
        robot.missionStart = false;
//         controlActive = false;
//         snprintf(s, MSL, "# loading hard coded mission %d (idx=%d)\r\n", mission, mission - 1);
//         usb.send(s);
      }
      break;
    case 1:
      if (userMission.missionTime > 0.5) // wait for finger away from button
      { // go to next state
        bool isOK;
        userMission.missionTime = 0.0;
        missionState = 0;
        backToNormal = mission > 0;
//         snprintf(s, MSL, "# loading hard coded mission %d (idx=%d) now (missionState=%d).\r\n", mission, mission - 1, missionState);
//         usb.send(s);
        // Load hard configuration
        isOK = eeConfig.hardConfigLoad(mission - 1, false);
        //
//         snprintf(s, MSL, "# implement as mission %d (OK=%d)\r\n", mission, isOK);
//         usb.send(s);
        // start the newly loaded mission
        if (isOK)
        {
          userMission.missionStop = false;
          robot.missionStart = true;
//           control.backToNormal = back;
//           usb.send("# UControl::mission_hard_code - set robot.missionStart = true\n");
        }
      }
      break;
    default:
      break;
  }
//   snprintf(s, MSL, "# UControl::mission_hard_code: missionState=%d\n", missionState);
//   usb.send(s);
  return isOK;
}

///////////////////////////////////////////////////////////////////////


/**
 * run mission from user lines */
void UControl::user_mission()
{
//   const int MSL = 160;
//   char s[MSL];
  bool irP = false;
  bool lsP = false;
  bool chirpExtraLog = false;
  switch (missionState)
  {
    case 0: // wait for start button
      display.useDisplay = true;
      if (robot.button or robot.missionStart)
      { // goto next state
        control.initControl();
        userMission.missionTime = 0.0;
        userMission.missionStop = false;
        // chirp must be reset (used or not)
        chirpReset(logger.logInterval);
        // log interval is number of values each 360 deg of chirp (3-5 or so).
        //         controlActive = false;
        if (robot.button)
        { // there may be more button-presses
          missionState = 6;
          usb.send("# UControl::user_mission: should never be here.\n\t");
        }
        else
        { // just shift to run-state (i.e. 2)
//           usb.send("# UControl::user_mission: state 0 -> state 1 (starting)\n");
          missionState = 1;
        }
        // tell bridge (and others) that a green button is pressed
        usb.send("event 33\n");
        usb.send("# UControl::user_mission: send event 33 (start mission)\n");
        userMission.getPowerUse(&lsP, &irP, &chirpExtraLog);
        if (irP)
        { // takes time to get first value
          usb.send("# IR power on\n");
          irdist.setIRpower(true);
        }
        if (lsP)
        { // turns on when used - that is enough
          // missin end will no longer turn it off
          ls.lineSensorOn = true;
          ls.lsEdgeValidCnt = 0;
          ls.crossingLineCnt = 0;
          usb.send("# Line sensor turned on\n");
        }
        if (chirpExtraLog)
        {
          usb.send("# chirp activated\n");
        }
        // enable or disable chirp log
        logger.logRowFlags[ULog::LOG_CHIRP] = chirpExtraLog;
        // reset log buffer (but do not start logging)
        logger.initLogStructure(50);
        // clear extra logger data (may not be used)
        if (false)
          memset(logger.dataloggerExtra, 0, sizeof(logger.dataloggerExtra));
        // first quit second should be quiet
        // after a RC session, motors are enabled
        motor.motorSetEnable(false, false);
        // send start event to client
        // this can be used as delayed start
//         usb.send("event 33\n");
      }
      break;
    case 1: // wait to allow release of button
//       usb.send("#user_mission:: state 1\n");
//       if (time > 1.0)
      { // init next state
        if (userMission.startMission())
        {
          // don't update display during a mission
          display.useDisplay = false;
          // shift to mission run state
          missionState = 2;
          // forget all old values
          resetControl();
          // make control active (if not already)
          controlActive = true;
//           motorSetEnable(true, true);
//           usb.send("# user_mission; (state 1) start mission -> state 2\n");
        }
        else
        {  // no user defined lines
          missionState = 9;
//           usb.send("# user_mission; no lines -> state 9\n");
          break;
        }
//         userMission.getPowerUse(&lsP, &irP);
//         if (irP)
//         { // takes time to get first value
//           // usb.send("# IR power on\n");
//           setIRpower(true);
//         }
//         if (lsP)
//         { // turns on when used - that is enough
//           lineSensorOn = true;
//           // usb.send("# line sensor power on\n");
//         }
//         motorSetEnable(true, true);
      }
      break;
    case 2:
      // run user mission lines
      { // test finished, and move to next line if not finished
        bool theEnd = userMission.testFinished();
        if (theEnd or userMission.missionStop or robot.button)
        { // finished - mission end
          missionState=8;
          endTime = userMission.missionTime + 0.05;
          // debug
          debugMissionEnd = true;
//           snprintf(s, MSL, "# user_mission; state 2 (run) theEnd (TheEnd=%d, MissionStop=%d, buttton=%d -> state 8\n", theEnd, userMission.missionStop, robot.button);
//           usb.send(s);
          // debug end
        }
      }
      break;
    case 6:
//       usb.send("#user_mission:: state 6\n");
      if (userMission.missionTime > 0.1 and not robot.button)
      {
        missionState = 7;
//         usb.send("# user_mission state 6 button released -> state 7\n");
      }
      else if (userMission.missionTime > 1.2)
      {
        missionState = 1;
        if (missionButtonCount > 0)
        {
          mission = missionButtonCount;
          // debug
//           snprintf(s, MSL, "# user_mission; state 6 starting hard coded mission %d (> 0)\n", mission);
//           usb.send(s);
        }
      }
      break;
    case 7:
//       usb.send("#user_mission:: state 7\n");
      if (robot.button)
      {
        missionState = 6;
        missionButtonCount ++;
//         usb.send("# user_mission; state 7 button pressed again, -> state 6\n");
      }
      else if (userMission.missionTime > 1.2)
      {
        missionState = 1;
//         usb.send("# user_mission; state 7 button time > 1.2, -> state 1\n");
        if (missionButtonCount > 0)
        {
          mission = missionButtonCount;
          // debug
//           snprintf(s, MSL, "## user_mission; state 7 starting mission %d\n", mission);
//           usb.send(s);
        }
      }
      break;
    case 8:
      // stop (start) button pressed - or just mission ended
//       usb.send("# user_mission (state 8)\n");
      // stop movement, and
      mission_vel_ref = 0;
      // wait for start button to be released (or 0.5 second)
      if (not robot.button and userMission.missionTime > endTime)
      { // to default state - stop
//         usb.send("# user_mission; state 8 (end) -> 9 the default end!\n");
        missionState = 9;
        userMission.stopMission();
      }
      break;
    default:
//       usb.send("# the end! (mission stop)\n");
      userMission.missionStop = true;
      usb.send("event 0\n");
      usb.send("# UControl::user_mission; mission stopped - send event 0\n");
      break;
  }
}



void UControl::eePromSaveCtrl()
{ // must be called in in right order
  eeConfig.pushByte(mission);
  ctrlVelLeft->eePromSave();
  ctrlVelRight->eePromSave();
  ctrlTurn->eePromSave();
  ctrlWallVel->eePromSave();
  ctrlWallTurn->eePromSave();
  ctrlPos->eePromSave();
  ctrlEdge->eePromSave();
  ctrlBal->eePromSave();
  ctrlBalVel->eePromSave();
  ctrlBalPos->eePromSave();
}

void UControl::eePromLoadCtrl()
{ // must be called in in right order
  mission = eeConfig.readByte();
  if (eeConfig.isStringConfig())
    // anny mission must run as mission 0
    mission = 0;
//   usb.send("# loaded mission type\n");
  ctrlVelLeft->eePromLoad();
  ctrlVelRight->eePromLoad();
  ctrlTurn->eePromLoad();
  ctrlWallVel->eePromLoad();
  ctrlWallTurn->eePromLoad();
  ctrlPos->eePromLoad();
  ctrlEdge->eePromLoad();
  ctrlBal->eePromLoad();
  ctrlBalVel->eePromLoad();
  ctrlBalPos->eePromLoad();
}


void UControl::setRegulatorInOut()
{ // set initial input and output for regulators
  // turn controllers
  ctrlWallVel->setInputOutput(&mission_wall_vel_ref, irdist.irDistance, &ctrl_vel_ref);
  ctrlWallTurn->setInputOutput(&mission_wall_ref, irdist.irDistance, vel_ref_red_turn, 0.3);
  // left and right line valid is now one value only
  ctrlEdge->setInputOutput(&mission_line_ref, &ls.lsLeftSide, &ls.lsRightSide, &ls.lsEdgeValid/*, &lseRightValid*/, 
                           vel_ref_red_turn, &mission_line_LeftEdge);
  ctrlTurn->setInputOutput(&ctrl_turn_ref /* &mission_turn_ref*/, &encoder.pose[2], vel_ref_red_turn);
  // position controller
  ctrlPos->setInputOutput(&mission_pos_ref, &encoder.distance, &misStartDist, &ctrl_vel_ref);
  // balance controllers
  ctrlBalPos->setInputOutput(&mission_pos_ref, &encoder.distance, &misStartDist, &ctrl_vel_ref);
  ctrlBalVel->setInputOutput(&ctrl_vel_ref, encoder.wheelVelocityEst, &balanceTiltRef);
  ctrlBal->setInputOutput(&balanceTiltRef, &encoder.pose[3], &balanceVelRef, &imu2.gyroTiltRate);
  // wheel velocity control
  ctrlVelLeft->setInputOutput(&vel_ref[0], &encoder.wheelVelocityEst[0], &motor.motorVoltage[0]);
  // wait with implementing motor voltage limit, to allow turning at all times
  // see tick on how this is implemented
  ctrlVelLeft->setDoImplementOutputLimitFlag(false);
  ctrlVelRight->setInputOutput(&vel_ref[1], &encoder.wheelVelocityEst[1], &motor.motorVoltage[1]);
  ctrlVelRight->setDoImplementOutputLimitFlag(false);
}


void UControl::resetControl()
{
  ctrlVelLeft->resetControl();
  ctrlVelRight->resetControl();
  ctrlTurn->resetControl();
  ctrlWallVel->resetControl();
  ctrlWallTurn->resetControl();
  ctrlPos->resetControl();
  ctrlEdge->resetControl();
  ctrlBal->resetControl();
  ctrlBalVel->resetControl();
  ctrlBalPos->resetControl();
  vel_ref[0] = 0;  
  vel_ref[1] = 0;
  vel_ref_pre_bal[0] = 0;  
  vel_ref_pre_bal[1] = 0;  
  vel_ref_red_turn[0] = 0;
  vel_ref_red_turn[1] = 0;
  // and line sensor
  ls.lsPostProcess();
}


void UControl::initControl()
{
  ctrlVelLeft->initControl();
  ctrlVelRight->initControl();
  ctrlTurn->initControl();
  ctrlWallVel->initControl();
  ctrlWallTurn->initControl();
  ctrlPos->initControl();
  ctrlEdge->initControl();
  ctrlBal->initControl();
  ctrlBalVel->initControl();
  ctrlBalPos->initControl();  
}


void UControl::resetTurn()
{
  vel_ref_red_turn[0] = 0;
  vel_ref_red_turn[1] = 0;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

bool once = false;
bool oldRemoteControlOn = false;
bool remoteCtrlEnded = false;


void UControl::tick(void)  
{
  // const int MSL = 150;
  // char s[MSL];
  bool relax = motor.allowRelax;
  tickCnt++;
  if (remoteControl)
  { // in remote control - possibly within a mission,
    // stop advancing mission
    if (not oldRemoteControlOn)
    {
      oldRemoteControlOn = true;
      usb.send("# UControl::controlTick: Shift to remote control\n");
    }
    relax = false;
  }
  else if (mission > 0 and not backToNormal) 
  { // may be start of hard coded mission
    hardMission = mission;
    mission_hard_coded();
  }
  else
  { // current mission - or prepare for hard coded mission
    user_mission();
  }
  //
  if (not remoteControl and oldRemoteControlOn)
  { // handling end of remote control state
    usb.send("# UControl::controlTick: Shift to auto (away from remote control)\n");
    if (controlActive)
      usb.send("# UControl::controlTick: control active\n");
    remoteCtrlEnded = true;
    oldRemoteControlOn = false;
  }
  if (/*not missionStop and*/ controlActive)
  { // control state for heartbeat status (and GUI)
    bool limitIsActive = ctrlVelLeft->outLimitUsed or
                         ctrlVelRight->outLimitUsed or
                         // rateLimitUsed or
                         motor.relax;
    if (not remoteControl)
      controlState = hardMission + 10;
    // ready to do control loops
//     if (tickCnt % 300 == 1)
//     {
//       const int MSL = 90;
//       char s[MSL];
//       snprintf(s, MSL, "# UControl::tick: mission %d, hard=%d, back=%d, ctlState=%d, %s\n", mission, hardMission, backToNormal, controlState, missionName[hardMission]);
//       usb.send(s);
//     }
    // use turn ref as is in most cases
    ctrl_turn_ref = mission_turn_ref;
    if (chirpRun)
    { // we are using chirps, so calculate current addition to control value
      // amplitude limitation
      relax = false;
      float dv = fabs(encoder.wheelVelocityEst[0] - mission_vel_ref);
      if (chirpDestination == 0 and dv > chirpVelMax)
      { // new max amplitude
        chirpVelMax = dv;
      }
//       else if (chirpDestination == 1 and fabs(imuGyro[2] * gyroScaleFac) > chirpGyroTurnrateMax)
//       { // new max turnrate
//         chirpGyroTurnrateMax = fabs(imuGyro[2] * gyroScaleFac);
//       }
      // phase change in sample time
      double dp = chirpFrq * robot.SAMPLETIME;
      // new phase angle
      chirpAngle += dp;
      // fold
      if (chirpAngle > 2*M_PI)
      { // a full cycle
        chirpLogQuadrant = 1;
        chirpAngle  -= 2*M_PI;
      }
      if (chirpAngle > 2 * M_PI * chirpLogQuadrant / float(chirpLogInterval))
      { // passed a log quadrant (or 2 PI)
        chirpLogQuadrant++;
        // tell log system to log
        chirpLog = true;
        // dectrase frequency at every log sample
        chirpFrq *= chirpFrqRate;
        if (chirpDestination == 0)
        {
          if (chirpVelMax > 0.5 or chirpMotorVoltMax > 7.0)
          { // reduce chirp amplitude and reset amplitude
            chirpMotorVoltMax = 5.0;
            chirpVelMax = 0.3;
            chirpAmplitude *= 0.99;
          }
        }
      }
      chirpValue = chirpAmplitude * cos(float(chirpAngle));
    }
    //
    if (remoteControl)
    { // velocity comes from remote
      ctrl_vel_ref = remoteControlVel;
    }
    else
    { // velocity comes from mission
      ctrl_vel_ref = mission_vel_ref;
      if (not mission_pos_use and relax)
        relax = fabs(ctrl_vel_ref) < 0.01;
      if (chirpRun)
      { // modify velocity or turn reference
        if (chirpDestination == 0)
          ctrl_vel_ref += chirpValue;
        else if (chirpDestination == 1)
          ctrl_turn_ref += chirpValue;
      }
      if (mission_irSensor_use and not mission_wall_turn)
      { // front distance is controlling velocity reference
        // balance or not
        ctrlWallVel->outLimitUsed = limitIsActive; // rateLimitUsed;
        ctrlWallVel->controlTick();
        // further away should increase speed
        ctrl_vel_ref *= -1.0;
        if (ctrl_vel_ref > mission_vel_ref)
          ctrl_vel_ref = mission_vel_ref;
        else if (regul_line_use and ctrl_vel_ref < 0)
          ctrl_vel_ref = 0;
        else if (ctrl_vel_ref < -mission_vel_ref)
          ctrl_vel_ref = -mission_vel_ref;
      }
      else if (mission_pos_use and not balance_active)
      { // position regulator provides velocity reference
        ctrlPos->outLimitUsed = limitIsActive or rateLimitUsed;
        ctrlPos->controlTick();
        if (relax)
          relax = fabs(ctrlPos->eu) < 0.01;
      }
    }
    // should balance be active
    if (balance_active)
    { // do balance control to control velocity reference
      // stop integrators if something is saturated
      // limitIsActive |= ctrlBalVel->outLimitUsed or
      //                  ctrlBal->outLimitUsed;
      //
      if (mission_pos_use and not remoteControl)
      { // position regulator active - in balance - providing velocity reference
        ctrlBalPos->outLimitUsed = limitIsActive or rateLimitUsed or
                                   ctrlBalVel->outLimitUsed or
                                   ctrlBal->outLimitUsed or
                                   ctrlBalPos->outLimitUsed;
        ctrlBalPos->controlTick();
        if (relax)
          relax = fabs(ctrlBalPos->eu) < 0.01;
      }
      // better results without
      // if (relax)
      //   // do not allow relaxing with tilt in motion
      //   relax = fabs(encoder.pose[3]) < 0.15; // ~10 deg
      //
      // disable integrators, if either balance velocity or balance output is saturated
      ctrlBalVel->outLimitUsed = limitIsActive or ctrlBal->outLimitUsed;
      // velocity in balance
      ctrlBalVel->controlTick();
      // balance
      // stop also balance integrator if
      ctrlBal->outLimitUsed = limitIsActive; // rateLimitUsed;
      ctrlBal->controlTick();
    }
    //
    // one of 4 turn possibilities
    if (remoteControl)
    { // turning comes from remote, use symmetric turn mode
      vel_ref_red_turn[0] = remoteControlVelDif/2.0;
      vel_ref_red_turn[1] = -remoteControlVelDif/2.0;
    }
    else if (mission_turn_do)
    { // fixed radius turn
      relax = false;
      if (once)
      {
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# turn control: velref=%g, tr=%g, turn_radius=%g\n", mission_vel_ref, mission_tr_turn, mission_turn_radius);
        usb.send(s);
        once = false;
      }
//    if ((mission_tr_turn >= 0 and mission_vel_ref >= 0) or (mission_tr_turn < 0 and mission_vel_ref < 0))
      if ((mission_tr_turn * mission_vel_ref) >= 0) // or (mission_tr_turn < 0 and mission_vel_ref < 0))
      { // regulate left wheel (relative to right wheel)
        float vel = encoder.wheelVelocityEst[1];
        vel = vel_ref[1];
          // calculate velocity reduction (may be more than actual velocity)
          vel_ref_red_turn[0] = vel * encoder.odoWheelBase / (mission_turn_radius + encoder.odoWheelBase / 2.0);
          vel_ref_red_turn[1] = 0;
      }
      else
      { // regulate right wheel and change sign in turn radius
        float vel = encoder.wheelVelocityEst[0];
        vel = vel_ref[0];
        vel_ref_red_turn[1] = vel * encoder.odoWheelBase / (mission_turn_radius + encoder.odoWheelBase / 2.0);
        vel_ref_red_turn[0] = 0;
      }
    }
    else if (regul_line_use)
    {  // turn using line edge controller
      ctrlEdge->outLimitUsed = rateLimitUsed;
      ctrlEdge->controlTick();
      if (relax)
        relax = fabs(ctrlEdge->eu) < 0.05;
    }
    else if (mission_irSensor_use > 0 and mission_wall_turn)
    {  // turn using wall follow controller
      ctrlWallTurn->outLimitUsed = rateLimitUsed;
      ctrlWallTurn->controlTick();// regulator_wall();
      relax = false;
    } 
    else
    { // try to keep current reference heading
      ctrlTurn->outLimitUsed = rateLimitUsed;
      ctrlTurn->controlTick();
      if (relax)
        relax = fabs(ctrlTurn->eu) < 0.02;
    }
    //
    // combine to wheel velocity ref
    float  v0 = -vel_ref_red_turn[0];
    float  v1 = -vel_ref_red_turn[1];
    if (not balance_active)
    { // if not in balance nor position control, 
      // then there is a velocity reference needed
      v0 += ctrl_vel_ref;
      v1 += ctrl_vel_ref;
      balanceVelRef = 0;
    }
    else if (true)
    { // test
      // apply balance before acc limit
      v0 += balanceVelRef;
      v1 += balanceVelRef;
    }
    if (rateLimitUse and not remoteControl)
    { // input should be rate limited - acceleration limit for a velocity controller
      float accSampleLimit = rateLimit * robot.SAMPLETIME;
      float d0 = v0 - vel_ref_pre_bal[0];
      float d1 = v1 - vel_ref_pre_bal[1];
      float dabs = fabs(d0) - fabs(d1);
      bool acc0go = true;
      bool acc1go = true;
      if (d0 * d1 > 0)
      { // both wheels is to accelerate in same
        // direction
        // wait with accelerating one of the wheels
        // until they are close to same speed (1cm/sec)
        if (dabs > 0.01)
          acc1go = false;
        else if (dabs < -0.01)
          acc0go = false;
      }
      // left wheel
      if (acc0go)
      { // left wheel acc limit
        rateLimitUsed = true;
        if (d0 > accSampleLimit)
          vel_ref_pre_bal[0] += accSampleLimit;
        else if (d0 < -accSampleLimit)
          vel_ref_pre_bal[0] -= accSampleLimit;
        else
        {
          vel_ref_pre_bal[0] = v0;
          rateLimitUsed = false;
        }
      }
      // right wheel
      if (acc1go)
      { // right wheel acc limit
        if (d1 > accSampleLimit)
          vel_ref_pre_bal[1] += accSampleLimit;
        else if (d1 < -accSampleLimit)
          vel_ref_pre_bal[1] -= accSampleLimit;
        else
        {
          vel_ref_pre_bal[1] = v1;
          rateLimitUsed = false;
        }
      }
      if (fabs(v1 - v0) < 0.01)
      {
        float vadif = fabs(vel_ref_pre_bal[0] - vel_ref_pre_bal[1]);
        if (vadif < 0.01 and (not acc0go or not acc1go))
        { // towards straight and vel_ref close
          // so make equal
          if (true)
          {
            if (not acc1go)
              vel_ref_pre_bal[0] = vel_ref_pre_bal[1];
            else if (not acc0go)
              vel_ref_pre_bal[1] = vel_ref_pre_bal[0];
          }
        }
      }
    }
    else
    { // no limit - use as is
      rateLimitUsed = false;
      vel_ref_pre_bal[0] = v0;
      vel_ref_pre_bal[1] = v1;
    }
    // add balance action - with no rate limit
    // test
    // with acc limit including balance
    if (false)
    { // old - no limit
      vel_ref[0] = vel_ref_pre_bal[0] + balanceVelRef;
      vel_ref[1] = vel_ref_pre_bal[1] + balanceVelRef;
    }
    else
    {
      vel_ref[0] = vel_ref_pre_bal[0]; // + balanceVelRef;
      vel_ref[1] = vel_ref_pre_bal[1]; // + balanceVelRef;
    }
    //
    // implement desired velocity for wheels
    








    // UNCOMMMENT FOR PID CONTROL
    // result is in motor.motorVoltage[0] and [1]

    // ctrlVelLeft->controlTick();
    // ctrlVelRight->controlTick();
    
    
  
    
    
    
    
    
    
    
    if (relax)
    {
      relax = fabs(encoder.wheelVelocityEst[0]) < 0.01 and
              fabs(encoder.wheelVelocityEst[1]) < 0.01;
      // if (not motor.relax and relax)
      // {
      //   snprintf(s, MSL, "# time=%.3f, to relax\n", userMission.missionTime);
      //   usb.send(s);
      // }
      // save conclusion as global variable
      motor.relax = relax;
    }
    else
    {
      // if (motor.relax)
      // {
      //   snprintf(s, MSL, "# time=%.3f, to not relax\n", userMission.missionTime);
      //   usb.send(s);
      // }
      motor.relax = false;
    }
    // if (tickCnt % 10 == 0)
    // {
    // snprintf(s, MSL, "# time=%.3f, relax %d, vel=%.3f, eTurn=%.3f, VelRef=%.3f, balvelLim=%d, balLim=%d, limitAll=%d, mRelax=%d\n",
    //          userMission.missionTime,
    //          relax,
    //          encoder.wheelVelocityEst[0],
    //          ctrlTurn->eu,
    //          ctrl_vel_ref,
    //          ctrlBalVel->outLimitUsed,
    //          ctrlBal->outLimitUsed,
    //          limitIsActive,
    //          motor.relax);
    // usb.send(s);
    // }
    //
    
    // test for overload shutdown.
    if (ctrlVelLeft->outLimitUsed or ctrlVelRight->outLimitUsed)
    { // actual limit is not implemented in
      // velocity control, but both motors are reduced to allow
      // steering even when there is an overload
      float maxv = fmaxf(fabsf(motor.motorVoltage[0]), fabsf(motor.motorVoltage[1]));
      float reduc = ctrlVelLeft->getOutLimit() / maxv;
      motor.motorVoltage[0] *= reduc;
      motor.motorVoltage[1] *= reduc;
      // 
      motor.overloadCount++;
    }
    // else
    motor.overloadCount = 0;
    
    
    // NN control. Overwrites motor voltage if control is active.
    nn.tick();

    
  }
  else
    controlState = 0;
  //
  // stop control and motors
  if ((missionState > 0 and userMission.missionStop) or (remoteCtrlEnded and missionState == 0))
  { // here once, when mission (or GUI) sets stop-flag
    motor.motorVoltage[0] = 0;
    motor.motorVoltage[1] = 0;
    mission_turn_do = false;
    mission_vel_ref = 0;
    mission_wall_vel_ref = 0;
    mission_wall_ref = 0;
    mission_line_ref = 0;
    mission_pos_ref = 0;
    mission_pos_use = false;
    mission_irSensor_use = false;
    controlActive = false;
    motor.motorSetEnable(false, false);
    logger.stopLogging();
    missionLineNum = 0;
    robot.missionStart = false;
    userMission.missionStop = false;
    usb.send("# message ** Mission stopped\r\n");
    missionState = 0;
    missionButtonCount = 0;
    remoteCtrlEnded = false;
    // enable idle collection of current data 
    // to calibrate zero current (at next mission start)
    motor.motorPreEnabled = true;
    if (false)
    { // due to Robobot with often ending missions 
      // do not turn off these sensors
      irdist.setIRpower(false);
      ls.lineSensorOn = false;
      // and leave servos engaged
      servo.releaseServos();
    }
    hardMission = 0;
    if (backToNormal)
    {
      eeConfig.eePromLoadStatus(false);
      backToNormal = false;
      usb.send("# UControl::tick: back to normal\n");
      display.useDisplay = true;
    }
    chirpRun = false;
  }
}

void UControl::chirpReset(int logInterval)
{
  chirpAngle = 0.0;
  chirpLogInterval = logInterval;
  chirpLogQuadrant = 0;
  chirpRun = false;
  chirpMinimumFrequency  = M_PI; // rad/s
//   tickCnt = 0;
}

void UControl::chirpStart(uint8_t chirp, bool useHeading)
{
  chirpReset(logger.logInterval);
  chirpAmplitude = chirp/100.0;
  chirpAngle = M_PI/2.0; // start at value 0 (cos(90 deg))
  chirpRun = chirp > 0;
//   chirpGyroTurnrateMax = 0;
  chirpMotorVoltMax = 0;
//   chirpGyroTurnrateMax2 = 0;
//   chirpVelMax2 = 0;
  if (chirpRun)
  { // start with 250 Hz - if log-interval=4 - in radians/sec
    chirpFrq = 2.0 * M_PI * 1000.0 / chirpLogInterval;
    // logRowCount is number of measurements in log
    // logInterval is log sample time in ms
    // make sure to reach 
    const double endFrequency = 0.5 * 2 * M_PI; // (0.5Hz)
    // calculate factor to use at every sample time (each ms)
    // see Maple calculation in regbot/matlab/chirp/chirp_calculation.ws
//     chirpFrqRate = exp(-chirpLogInterval*(chirpFrq - endFrequency)/(1000.0 * logRowsCntMax * 2 * M_PI));
//     float endTime = -log(endFrequency/chirpFrq) * logRowsCntMax * 2 * M_PI / (chirpLogInterval * (chirpFrq - endFrequency));
    // changed to update frequency at log event
    // see Maple calculation in regbot/matlab/chirp/chirp_calculation2.ws
    chirpFrqRate = exp(log(endFrequency / chirpFrq)/logger.logRowsCntMax);
    //
    if (useHeading)
      chirpDestination = 1; // heading
    else
      chirpDestination = 0; // velocity
    // debug
    const int MSL = 150;
    char s[MSL];
    snprintf(s, MSL, "#chirp start f=%g, rate=%g, dest=%d, rows=%d, interval=%d, endTime=%g\n", 
             chirpFrq, chirpFrqRate, chirpDestination, logger.logRowsCntMax, chirpLogInterval, endTime);
    usb.send(s);
    // debug end
  }
  else
    usb.send("#chirp off\n");
}
