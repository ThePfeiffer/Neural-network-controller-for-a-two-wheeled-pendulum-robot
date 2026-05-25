/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
 *   jcan@dtu.dk            
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
 * aug 2024: new velocity estimate - less noise, but with a delay (~10ms at 15cm/s)  /jca)
 * */

#include <stdlib.h>
#include "main.h"
#include "uencoder.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
#include "ucontrol.h"
#include "umotortest.h"
#include "umotor.h"

UEncoder encoder;

void UEncoder::setup()
{ // input should be default, but pin PIN_RIGHT_ENCODER_B on HW41 fails
  attachInterrupt ( M1ENC_A, m1EncoderA, CHANGE );
  attachInterrupt ( M2ENC_A, m2EncoderA, CHANGE );
  attachInterrupt ( M1ENC_B, m1EncoderB, CHANGE );
  attachInterrupt ( M2ENC_B, m2EncoderB, CHANGE );
  // use hysteresis on input levels (to avoid extra trigger on slow transitions)
  *digital_pin_to_info_PGM[M1ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M1ENC_B].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_B].pad |= IOMUXC_PAD_HYS;
  // data subscription
  addPublistItem("enc", "Get encoder value 'enc m1 m2' (int32)");
  addPublistItem("pose", "Get current pose 'pose t x y h' (sec,m,m,rad)");
  addPublistItem("vel", "Get velocity 'left right' (m/s)");
  addPublistItem("conf", "Get robot conf (radius, radius, gear, pulsPerRev, wheelbase, sample-time, reversed)");
  addPublistItem("vem", "Get motor and wheel velocity 'left right left right' (rad/s)");
  addPublistItem("ene", "Get encoder error 'enc NANcount reversed M1err M2err' (int32)");
  addPublistItem("end1", "Encoder detail M1 'end1 e e e e d d d d vf v' e:edge count, d:delta time, vf:fast motor vel, v:motor vel");
  addPublistItem("end2", "Get encoder error 'enc NANcount reversed M1err M2err' (int32)");
  usb.addSubscriptionService(this);
  versioncpp = "$Id: uencoder.cpp 1746 2025-12-30 17:02:28Z jcan $";
}


void UEncoder::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Encoder settings -------\r\n");
  snprintf(reply, MRL, "# -- \tenc0 \tReset pose to (0,0,0)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tconfw rl rr g t wb \tSet configuration (radius gear encTick wheelbase)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tencrev R \tSet motortest reversed encoder (R=0 normal, R=1 reversed, is %d)\r\n", motortest.encoderReversed);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \t         \tNormal: motv 3 3 => left enc decrease, right enc increase.\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tencf F C\tUse fast estimate (F=0 delay but cleaner, F=1 faster, is %d, C=0 don't compensate, U=1 compensate, is=%d)\r\n", velEstFast, velEstFastCompensate);
  usb.send(reply);
}


bool UEncoder::decode(const char* buf)
{
  const int MSL = 200;
  char s[MSL];
  bool used = true;
  if (strncmp(buf, "enc0", 4) == 0)
  { // reset pose
    clearPose();
  }
  else if (strncmp(buf, "encd", 4) == 0)
  { // debug encoder print
    int n = eportCnt;
    snprintf(s, MSL, "encp %d\n", n);
    usb.send(s);
    for (int i = 0; i < n; i++)    
    {
      snprintf(s, MSL, "enct %d %d %d %d %d %f\n", i, eport[i][3], eport[i][0], eport[i][1], eport[i][2], edt[i]);
      usb.send(s);
    }
    eportCnt = 0;
  }
  else if (strncmp(buf, "confw ", 5) == 0)
  { // robot configuration
    const char * p1 = &buf[5];
    odoWheelRadius[0] = strtof(p1, (char**) &p1);
    odoWheelRadius[1] = strtof(p1, (char**) &p1);
    gear = strtof(p1, (char**) &p1);
    pulsPerRev = strtol(p1, (char**) &p1, 10);
    odoWheelBase = strtof(p1, (char**) &p1);
    // debug
    snprintf(s, MSL, "# got confw: r1=%g, r2=%g, G=%g, PPR=%d, WB=%g\n", odoWheelRadius[0],
             odoWheelRadius[1], gear, pulsPerRev, odoWheelBase);
    usb.send(s);
    // debug end
    if (pulsPerRev < 1)
      pulsPerRev = 1;
    if (gear < 1)
      gear = 1;
    if (odoWheelBase < 0.02)
      odoWheelBase = 0.02;
    anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  }
  else if (strncmp(buf, "encrev ", 7) == 0)
  { // robot configuration
    const char * p1 = &buf[7];
    motortest.encoderReversed = strtol(p1, (char**) &p1, 10);
  }
  else if (strncmp(buf, "encf ", 5) == 0)
  { // robot configuration
    const char * p1 = &buf[5];
    velEstFast = strtol(p1, (char**) &p1, 10);
    velEstFastCompensate = strtol(p1, (char**) &p1, 10);
  }
  else
    used = false;
  return used;
}

void UEncoder::sendData(int item)
{
  if (item == 0)
    sendEncStatus();
  else if (item == 1)
    sendPose();
  else if (item == 2)
    sendVelocity();
  else if (item == 3)
    sendRobotConfig();
  else if (item == 4)
    sendMotorVelocity();
  else if (item == 5)
    sendEncoderErrors();
  else if (item == 6)
    sendEncDetailM1();
  else if (item == 7)
    sendEncDetailM2();
}


void UEncoder::sendEncStatus()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "enc %lu %lu\r\n",
           encoder[0], encoder[1]);
  usb.send(s);
}

void UEncoder::sendEncoderErrors()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "ene %d %d %d %d\r\n",
           nanCnt,
           motortest.encoderReversed,
           errCntA[0][0] + errCntA[0][1] + errCntB[0][0] + errCntB[0][1],
           errCntA[1][0] + errCntA[1][1] + errCntB[1][0] + errCntB[1][1]);
  usb.send(s);
}

void UEncoder::sendRobotConfig()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "conf %.4f %.4f %.3f %u %.4f %.6f %d\r\n", odoWheelRadius[0], odoWheelRadius[1],
    gear, pulsPerRev, odoWheelBase, robot.SAMPLETIME, motor.motorReversed
  );
  usb.send(s);
}

void UEncoder::sendPose()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "pose %lu.%04lu %.3f %.3f %.4f %.4f\n",
           tsec, tusec/100, 
           pose[0], pose[1], pose[2], pose[3]);
  usb.send(s);
}

void UEncoder::sendVelocity()
{
  const int MSL = 130;
  char s[MSL];
  snprintf(s, MSL, "vel %lu.%03lu %.3f %.3f %.4f %.3f\n",
           tsec, tusec/1000, 
           wheelVelocityEst[0], wheelVelocityEst[1], robotTurnrate, robotVelocity);
  usb.send(s);
}

void UEncoder::sendMotorVelocity()
{
  const int MSL = 130;
  char s[MSL];
  snprintf(s, MSL, "vem %lu.%03lu %.1f %.1f %.3f %.3f\n",
           tsec, tusec/1000,
           motorVelocity[0], motorVelocity[1], wheelVelocity[0], wheelVelocity[1]);
  usb.send(s);
}

void UEncoder::sendEncDetailM(int m)
{
  const int MSL = 300;
  char s[MSL];
  snprintf(s, MSL, "end%d %lu.%03lu %d %d %d %d  %.3f %.3f %.3f  %.2f %.2f %.2f %.2f  %.0f %.0f %.3f %.3f  %.3f %.3f %.3f %.3f  %.3f %.3f %.3f %.3f  %.3f %.3f %.3f %.3f\r\n",
           m, tsec, tusec/1000,
           incrEnc[m][0], incrEnc[m][1], incrEnc[m][2], incrEnc[m][3],
           motorVelocityFast[m] / gear * odoWheelRadius[m],
           motorVelocityFastMod[m] / gear * odoWheelRadius[m],
           motorVelocity[m] / gear * odoWheelRadius[m],
           velAge[m][0], velAge[m][1], velAge[m][2], velAge[m][3],
           velDif[m][0], velDif[m][1], velDif[m][2], velDif[m][3],
           velDif[m][4], velDif[m][5], velDif[m][6], velDif[m][7],
           velDif[m][8], velDif[m][9], velDif[m][10], velDif[m][11],
           velDif[m][12], velDif[m][13], velDif[m][14], velDif[m][15]
  );
  usb.send(s);
}

void UEncoder::tick()
{ // Update pose estimates
  tickCnt++;
  // estimate sample time - average over about 50 samples
  uint32_t t_CPU = ARM_DWT_CYCCNT;
  int32_t dt = t_CPU - lastSample_CPU;
  lastSample_CPU = t_CPU;
  // filter over 50 samples (float)
  sampleTime_us = (sampleTime_us * 49 + float(dt) * CPU_us)/50.0;
  //
  // update motor velocity (in rad/sec before gear)
  updateVelocityEstimate();
  // should not happen
  if (isnan(pose[0]) or isnan(pose[1]) or isnan(pose[2]))
  {
    clearPose();
    nanCnt++;
  }
  // update odometry pose
  updatePose(tickCnt);
}

///////////////////////////////////////////////////////

void UEncoder::eePromSave()
{
  uint16_t flags = velEstFast |
                   velEstFastCompensate << 0x01;
  eeConfig.pushWord(flags);
  for (int m = 0; m < MOTOR_CNT; m++)
  { // all motors
    for (int i = 0; i < 16; i++)
    { // load velocity compensation for encoder edge combinations
      if (i  == 2 or i == 3 or (i >=6 and i <= 9) or i == 12 or i == 13)
      { // these are the valid 8 cobinations (4 forward and 4 reverse)
        eeConfig.pushFloat(velDif[m][i]);
      }
    }
    // debug
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# saved  fast enc values: fast=%d, fastComp=%d, comp[%d][12]=%g\r\n", velEstFast, velEstFastCompensate, m, velDif[m][12]);
    usb.send(s);
    // debug end
  }
  eeConfig.pushFloat(odoWheelRadius[0]);
  eeConfig.pushFloat(odoWheelRadius[1]);
  eeConfig.pushFloat(gear);
  eeConfig.pushWord(pulsPerRev);
  eeConfig.pushFloat(odoWheelBase);
}

void UEncoder::eePromLoad()
{
  uint16_t flags = 0;
  if (not eeConfig.isStringConfig())
  { // load from flash
    if (eeConfig.loadedRev > 16935)
    { // newer than SVN 1693 minor 5 - should have
      // encoder compensation
      flags = eeConfig.readWord();
      velEstFast = (flags & 0x01) > 0;
      velEstFastCompensate = (flags & 0x02) > 0;
      for (int m = 0; m < MOTOR_CNT; m++)
      { // all motors
        for (int i = 0; i < 16; i++)
        { // load velocity compensation for encoder edge combinations
          if (i  == 2 or i == 3 or (i >=6 and i <= 9) or i == 12 or i == 13)
          { // these are the valid 8 cobinations (4 forward and 4 reverse)
            velDif[m][i] = eeConfig.readFloat();
          }
        }
        // debug
        // const int MSL = 200;
        // char s[MSL];
        // snprintf(s, MSL, "# loaded fast enc values: fast=%d, fastComp=%d, comp[%d][12]=%g\r\n", velEstFast, velEstFastCompensate, m, velDif[m][12]);
        // usb.send(s);
        // debug end
      }
    }
    // else
    //   usb.send("# did not load fast encoder values\r\n");
    odoWheelRadius[0] =  eeConfig.readFloat();
    odoWheelRadius[1] =  eeConfig.readFloat();
    gear = eeConfig.readFloat();
    pulsPerRev = eeConfig.readWord();
    // debug
    // const int MSL = 200;
    // char s[MSL];
    // snprintf(s, MSL, "# loaded PulsPerRef=%d after fastVelEst\r\n", pulsPerRev);
    // usb.send(s);
    // debug end
    odoWheelBase = eeConfig.readFloat();
    if (pulsPerRev < 1)
      pulsPerRev = 1;
    if (gear < 1)
      gear = 1;
    if (odoWheelRadius[0] < 0.001)
      odoWheelRadius[0] = 0.001;
    if (odoWheelRadius[1] < 0.001)
      odoWheelRadius[1] = 0.001;
    if (odoWheelBase < 0.01)
      odoWheelBase = 0.01;
    anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  }
  else
  { // skip robot specific items
    int skip = 3*4 + 2 + 4;
    if (eeConfig.loadedRev > 16935)
      skip += 2 + MOTOR_CNT*8*4;
    eeConfig.skipAddr(skip);
  }
}

void UEncoder::clearPose()
{
  pose[0] = 0;
  pose[1] = 0;
  pose[2] = 0;
  // pose[3] = 0; NB! tilt should NOT be reset
  encoder[0] = 0;
  encoder[1] = 0;
  encoderLast[0] = 0;
  encoderLast[1] = 0;
  distance = 0.0;
}


void UEncoder::updatePose(uint32_t loop)
{
  // wheel velocity in m/s
  if (velEstFast)
  { // wheel velocity on radians per second based on corrected fast method
    if (velEstFastCompensate)
    { // use the estimated edge to edge modification
      wheelVelocity[0] = -motorVelocityFastMod[0] /gear;
      wheelVelocity[1] = motorVelocityFastMod[1] /gear;
    }
    else
    { // use the un-modified estimate
      wheelVelocity[0] = -motorVelocityFast[0] /gear;
      wheelVelocity[1] = motorVelocityFast[1] /gear;
    }
  }
  else
  { // wheel velocity on radians per second
    wheelVelocity[0] = -motorVelocity[0] /gear;
    wheelVelocity[1] = motorVelocity[1] /gear;
  }
  wheelVelocityEst[0] = wheelVelocity[0] * odoWheelRadius[0];
  wheelVelocityEst[1] = wheelVelocity[1] * odoWheelRadius[1];
  // Turnrate in rad/sec
  robotTurnrate = (wheelVelocityEst[1] - wheelVelocityEst[0])/odoWheelBase ;
  // Velocity in m/s
  robotVelocity = (wheelVelocityEst[0] + wheelVelocityEst[1])/2.0;
  //
  // calculate movement and pose based on encoder count
  // encoder count is better than velocity based on time.
  // encoder position now
  uint32_t p1 = encoder[0];
  uint32_t p2 = encoder[1];
  // position change in encoder tics since last update
  int dp1 = (int32_t)p1 - (int32_t)encoderLast[0];
  int dp2 = (int32_t)p2 - (int32_t)encoderLast[1];
  // save current tick position to next time
  encoderLast[0] = p1;
  encoderLast[1] = p2;
  // position movement with forward as positive
  float d1 =  -dp1 * anglePerPuls / gear * odoWheelRadius[0];
  float d2 =   dp2 * anglePerPuls / gear * odoWheelRadius[1];
  // heading change in radians
  float dh = (d2 - d1) / odoWheelBase;
  // distance change in meters
  float ds = (d1 + d2) / 2.0;
  distance += ds;
  // add half the angle
  pose[2] += dh/2.0;
  // update pose position
  pose[0] += cosf(pose[2]) * ds;
  pose[1] += sinf(pose[2]) * ds;
  // add other half angle
  pose[2] += dh/2.0;
  // debug
  if (false and control.controlState > 0)
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# Pose:: t=%.4f, wvel=%g,%g; wvelEst=%g,%g, turnVel=%g, robVel=%g\n",
            userMission.missionTime,
            wheelVelocity[0], wheelVelocity[1],
            wheelVelocityEst[0], wheelVelocityEst[1],
             robotTurnrate, robotVelocity);
    usb.send(s);
  }
  // fold angle
  if (pose[2] > M_PI)
    pose[2] -= M_PI * 2;
  else if (pose[2] < -M_PI)
    pose[2] += M_PI * 2;

}

void UEncoder::updateVelocityEstimate()
{
  const float    one_sec_in_cpu  = F_CPU;
  const int32_t cpu_200ms = F_CPU/5;
  const uint32_t cpu_500ms = F_CPU/2;
  const int32_t cpu_2ms = F_CPU/500;
  // motor 1 velocity
  int j = active;
  active = (j + 1) % 2;
  float velSum[MOTOR_CNT] = {0};
  int velSumCnt[MOTOR_CNT] = {0};
  // float velSlowSum[MOTOR_CNT] = {0};
  // int velSlowSumCnt[MOTOR_CNT] = {0};
  // angle for full period of encoder
  const float app = anglePerPuls * 4;
  uint32_t timeStartEdge = 0;
  uint32_t timeEndEdge = 0;
  uint32_t timeStartNoEdge = 0;
  uint32_t nowCpu = ARM_DWT_CYCCNT;
  const float minMotorVel = 2.0; // radians/s on motor before gear.
  // bool overflow = false;
  //
  for (int m = 0; m < MOTOR_CNT; m++)
  { // save increment counter - debug
    for (int ab4 = 0; ab4 < 4; ab4++)
    { // just for debug
      dEncoder[m][ab4] = incrEncoder[m][j][ab4];
      // debug end
      if (incrEncoder[m][j][ab4] == 0)
      { // no increment since last
        incrEnc[m][ab4] = 0;
      }
      else
      { // use time since saved transition of same edge type
        uint32_t dt_cpu = transitionTime_cpu[m][j][ab4] - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < cpu_500ms)
        { // no overload, and valid timing ('incrEncoder' holds the sign)
          v = app * one_sec_in_cpu / dt_cpu * incrEncoder[m][j][ab4];
          velocityPart[m][ab4] = v;
          //
          // find also average delay time
          if (velSumCnt[m] == 0)
          { // first value
            timeStartEdge = lastTransitionTime_cpu[m][ab4];
            timeEndEdge = transitionTime_cpu[m][j][ab4];
          }
          else if (timeStartNoEdge < lastTransitionTime_cpu[m][ab4])
          { // more than one - use the newest
            timeStartEdge = lastTransitionTime_cpu[m][ab4];
            timeEndEdge = transitionTime_cpu[m][j][ab4];
          }
        }
        // save new transition time as last
        lastTransitionTime_cpu[m][ab4] = transitionTime_cpu[m][j][ab4];
        // edges used in this sample
        incrEnc[m][ab4] = incrEncoder[m][j][ab4];
        // prepare for next period
        incrEncoder[m][j][ab4] = 0;
        // use this velocity
        velSum[m] += v;
        velSumCnt[m]++;
      }
    }
    uint32_t delayAvg;
    if (velSumCnt[m] > 0)
    { // there were edges in this sample period
      // delay time in CPU clocks of newest data
        uint32_t dataPeriod = timeEndEdge - timeStartEdge;
        delayAvg = dataPeriod/2.0 + nowCpu - timeEndEdge;
        encoderDelay[m] = float(delayAvg)/one_sec_in_cpu;
        // motor velocity estimate in rad/s
        motorVelocity[m] = velSum[m] / velSumCnt[m];
    }
    else
    { // no edges in this sample
      // velocity estimate
      // use the fastest estimate that is slower than last
      // edge based velocity
      int32_t etime;
      // time since positive A edge
      int32_t maxTime = 1; // in CPU clocks
      float va; // absolute velocity
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find longest time since edge
        etime = nowCpu - lastTransitionTime_cpu[m][ab4];
        if (etime > maxTime)
          maxTime = etime;
      }
      va = app / (maxTime / one_sec_in_cpu);
      if (va < minMotorVel) // approx 5mm/sec for regbot
        va = 0;
      // velocity need update ('va' always positive)
      if (va < fabsf(motorVelocity[m]))
      { // keep old velocity if faster than this estimate
        if (motorVelocity[m] > 0)
          motorVelocity[m] = va;
        else
          motorVelocity[m] = -va;//velSlowSum[m] / velSlowSumCnt[m];
      }
      // avg delay is half time since last edge of newest data
      encoderDelay[m] = float(maxTime/2)/one_sec_in_cpu;
    }
    if (true or velEstFast)
    { // use 2 newest edges
      uint32_t newest = 0;
      uint32_t nextNewest = 0;
      int ab[2]{0};
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find the newest
        velAge[m][ab4] = (nowCpu - lastTransitionTime_cpu[m][ab4])/(one_sec_in_cpu / 1000);
        if (newest < lastTransitionTime_cpu[m][ab4])
        {
          newest = lastTransitionTime_cpu[m][ab4];
          ab[0] = ab4;
        }
      }
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find the next newest
        if (ab4 != ab[0] and nextNewest < lastTransitionTime_cpu[m][ab4])
        {
          nextNewest = lastTransitionTime_cpu[m][ab4];
          ab[1] = ab4;
        }
      }
      int32_t dataPeriod = newest - nextNewest;
      int32_t since = nowCpu - newest;
      delayAvg = dataPeriod/2.0 + since;
      if (velEstFast)
        encoderDelay[m] = float(delayAvg)/one_sec_in_cpu;
      bool calculated = false;
      // motor velocity estimate in rad/s
      if (since < 0 or since > cpu_200ms or dataPeriod <= 0)
      {  // too slow or time overflow
        motorVelocityFast[m] = 0;
      }
      else if (since > dataPeriod + cpu_2ms)
      { // use time since last edge
        motorVelocityFast[m] = anglePerPuls / (float(since)/one_sec_in_cpu);
      }
      else
      {  // use time of longest period
        motorVelocityFast[m] = anglePerPuls / (float(dataPeriod)/one_sec_in_cpu);
        calculated = true;
      }
      // add velocity sign
      if (encCCV[m])
        motorVelocityFast[m] *= -1;
      // save difference - first index, then values
      // index 2,3, 6,7,8,9, 12, 13 should be used only (3,6,8,13 for reverse)
      velDif[m][0] = ab[0];
      velDif[m][1] = ab[1];
      int e = ab[0]*4 + ab[1];
      if (fabs(motorVelocity[m]) > minMotorVel and calculated)
        velDif[m][e] = velDif[m][e] * (0.95) + (motorVelocity[m] - motorVelocityFast[m])/motorVelocity[m] * 0.05;
      motorVelocityFastMod[m] = motorVelocityFast[m] * (1 + velDif[m][e]);
    }

  }
}


void UEncoder::encoderInterrupt(int m, bool encA)
{ // get interrupt timing in CPU clocks
  uint32_t edge_cpu = ARM_DWT_CYCCNT;
  uint8_t pA, pB;
  // get encoder values for this motor
  pA = digitalReadFast(encApin[m]);
  pB = digitalReadFast(encBpin[m]);
  bool err = false;
  // edge index: A-up = 0, A-down = 1, B-up = 2, B-down = 3
  int ab4;
  if (encA)
  { // encoder pin A interrupt
    encCCV[m] = pA == pB;
    err = pA == lastA[m];
    lastA[m] = pA;
    if (err)
      errCntA[m][pA]++;
    if (pA)
      ab4 = 0;
    else
      ab4 = 1;
  }
  else
  { // encoder pin B interrupt
    encCCV[m] = pA != pB;
    err = pB == lastB[m];
    lastB[m] = pB;
    if (err)
      errCntB[m][pB]++;
    if (pB)
      ab4 = 2;
    else
      ab4 = 3;
  }
  if (err)
  { // this was a spurious interrupt, ignore
    // encoder value didn't change
    return;
  }
  // use this set of data to save values
  int j = active;
  if (encCCV[m])
  {
    encoder[m]--;
    // and within sample period
    incrEncoder[m][j][ab4]--;
  }
  else
  {
    encoder[m]++;
    // and within sample period
    incrEncoder[m][j][ab4]++;
  }
  transitionTime_cpu[m][j][ab4] = edge_cpu;
}

//////////////////////////////////////////////////////////////

void m1EncoderA()
{ // motor 1 encoder A change
    encoder.encoderInterrupt(0,true);
    encoder.intCnt++;
}

void m2EncoderA()
{ // motor 2 encoder A
    encoder.encoderInterrupt(1, true);
    encoder.intCnt++;
}

void m1EncoderB()
{ // motor 1 encoder pin B
    encoder.encoderInterrupt(0, false);
    encoder.intCnt++;
}

void m2EncoderB()
{ // motor 2 encoder pin B
    encoder.encoderInterrupt(1, false);
    encoder.intCnt++;
}
