/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
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

#include <stdlib.h>
#include "main.h"
#include "umotor.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
#include "ucontrol.h"
#include "usubss.h"
#include "ulog.h"
#include "uencoder.h"

UMotor motor;

void UMotor::setup()
{
  analogWriteResolution(12); /// resolution (12 bit)
  // find offset for motor current
  motorPreEnabled = true;
  //
  motorSetEnable(0,0);
  #if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  pinMode(PIN_LEFT_DIR,OUTPUT); // motor 1 IN1 (LEFT_IN_+)
  pinMode(PIN_RIGHT_DIR,OUTPUT); // motor 2 IN1 (RIGHT_IN_+)
  pinMode(PIN_LEFT_PWM,OUTPUT); // motor 1 IN2 (LEFT_IN_-)
  pinMode(PIN_RIGHT_PWM,OUTPUT); //motor 2 IN2 (RIGHT_IN_-)
  // PWM is set to output when 
//   setPWMfrq(PWMfrq);
  /**
   *  analogWriteFrequency(PIN_LEFT_DIR, PWMfrq); /// frequency (Hz)
   *  analogWriteFrequency(PIN_LEFT_PWM, PWMfrq); /// frequency (Hz)
   *  analogWriteFrequency(PIN_RIGHT_DIR, PWMfrq); /// frequency (Hz)
   *  analogWriteFrequency(PIN_RIGHT_PWM, PWMfrq); /// frequency (Hz)
   * */
//   analogWriteFrequency(PIN_LEFT_DIR, PWMfrq);
//   analogWriteFrequency(PIN_RIGHT_DIR, PWMfrq);
//   analogWriteFrequency(PIN_LEFT_PWM, PWMfrq);
//   analogWriteFrequency(PIN_RIGHT_PWM, PWMfrq);
  motorSetPWM(0,0);
  if (externalDriver and robot.robotHWversion == 9)
  { // fault pin is now IN2 on motor driver
    pinMode(PIN_LEFT_FAULT,OUTPUT); // motor 1 IN2
    pinMode(PIN_RIGHT_FAULT,OUTPUT); // motor 2 IN2
    ad.setExternalBatteryVoltageSensor(A6);
    robot.setTempAndPowerManagementUse(true);
  }
  #else
  switch (robot.robotHWversion)
  {
    case 1: // no used anymore
      break;
    case 2: // probably not used, but same motherboard as 5
    case 5: // with new satellite boards for power and IR
      pinMode(SLEW,OUTPUT); // slewrate hight
      digitalWrite(SLEW,HIGH); // slewrate - always high
      pinMode(M1PWM,OUTPUT); // motor 1 PWM 
      pinMode(M1DIR,OUTPUT); // motor 1 direction
      pinMode(M12DIS,OUTPUT); // disable motor 1 and 2
      //       pinMode(M2DIR1,OUTPUT); // direction (hardware < 3 only)
      pinMotor2Pwm = M2PWM1;
      pinMode(pinMotor2Pwm, OUTPUT);
      // slow motor controller
      analogWriteFrequency(pinMotor2Pwm, 17000); // motor 2 (right)
      analogWriteFrequency(M1PWM, 17000); // motor 1 (left)
      pinMotor2Dir = M2DIR1;
      pinMode(pinMotor2Dir, OUTPUT);
      // write to motor controller pins
      pinMode(M1DIS1, OUTPUT);
      pinMode(M2DIS1, OUTPUT);
      digitalWrite(M1DIS1, motorEnable[0]); 
      digitalWrite(M2DIS1, motorEnable[1]);
      break;
    case 3:
      //       pinMode(M1DIS1,OUTPUT); // disable motor 1
      pinMode(M1PWM,OUTPUT); // motor 1 PWM 
      pinMode(M1DIR,OUTPUT); // motor 1 direction
      pinMode(M12DIS,OUTPUT); // disable motor 1 and 2
//       pinMode(M2DIR1,OUTPUT); // direction (hardware < 3 only)
      pinMotor2Pwm = M2PWM3;
      pinMode(pinMotor2Pwm, OUTPUT);
      // slow motor controller
      analogWriteFrequency(pinMotor2Pwm, 17000); // motor 2 (right)
      analogWriteFrequency(M1PWM, 17000); // motor 1 (left)
      pinMotor2Dir = M2DIR3;
      pinMode(pinMotor2Dir, OUTPUT);
      // write to motor controller pins
//       if (false)
//       { // wrong - pre version 3.3?
//         digitalWrite(M1DIS1, motorEnable[0]); 
//         digitalWrite(M2DIS1, motorEnable[1]);
//       }
//       else
      digitalWrite(M12DIS, motorEnable[0]);
      break;
      
      usb.send("# setup motor controller 1,2,3,5\n");
    case 4:
    case 6:
      pinMode(M1PWM,OUTPUT); // motor 1 PWM 
      pinMode(M1DIR,OUTPUT); // motor 1 direction
      pinMode(M12DIS,OUTPUT); // disable both motors pin
      pinMode(M2DIR3,OUTPUT); // direction (hardware 3 only) pin
      pinMode(M2PWM3,OUTPUT); //OUTPUT); // PWM signal right motor (hardware 3 only) pin
//       if (robot.robotHWversion == 4)
//         PWMfrq 
//         analogWriteFrequency(M2PWM3, 25000); // small motor controller (both)
//       else
//         analogWriteFrequency(M2PWM3, 17000); // big motor controller (both)
      pinMotor2Dir = M2DIR3;
      pinMotor2Pwm = M2PWM3;
      // write to motor controller pins
      digitalWrite(M12DIS, motorEnable[0]);
      break;
    case 9:
      if (externalDriver)
      { // fault pin is now IN2 on motor driver
        pinMode(PIN_LEFT_FAULT,OUTPUT); // motor 1 IN2
        pinMode(PIN_RIGHT_FAULT,OUTPUT); // motor 2 IN2
        ad.setExternalBatteryVoltageSensor(A6);
      }
      break;
    default:
      break;
  }
  #endif
  // PWM is set to output when 
//   analogWriteFrequency(PIN_LEFT_DIR, PWMfrq);
//   analogWriteFrequency(PIN_RIGHT_DIR, PWMfrq);
//   analogWriteFrequency(PIN_LEFT_PWM, PWMfrq);
//   analogWriteFrequency(PIN_RIGHT_PWM, PWMfrq);
  setPWMfrq(PWMfrq);
  motorSetPWM(0,0);
  //
  if (setupCnt == 0)
  { // only once
    addPublistItem("mot", "Get motor voltage 'mot m1(V) m2(V) vel_ref1(m/s) vel_ref2(m/s) reversed relaxing'");
    addPublistItem("motpwm", "Get motor direction and PWM 'motpwm dir1 pwm1 dir2 pwm2'");
    usb.addSubscriptionService(this);
  }
  setupCnt++;
  versioncpp = "$Id: umotor.cpp 1768 2026-04-24 07:39:28Z jcan $";
}


/**
 * e2 used on hardware < 3 only */
void UMotor::motorSetEnable(uint8_t e1, uint8_t e2)
{ // sleep or un-sleep motors
  // debug
  if (false)
  {
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# UMotor::motorSetEnable(%d,%d), moPreEn=%d, moEn[0]=%d, moEn[1]=%d\n", e1, e2, motorPreEnabled, motorEnable[0], motorEnable[1]);
    usb.send(s);
  }
  // debug end
  if (motorPreEnabled and (e1 or e2) and not (motorEnable[0] or motorEnable[1]))
  { // switch off current zero offset calculation
    motorPreEnabled = false;
    motorPreEnabledRestart = true;
  }
  // reset overload
  if (e1 and not motorEnable[0])
    overloadCount = 0;
  // enable motors (or disable)
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  //
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  if (motorEnable[0])
  {// out of sleep
    if (not externalDriver)
    {
      pinMode(PIN_LEFT_DIR,OUTPUT);
      pinMode(PIN_LEFT_PWM,OUTPUT);
      digitalWriteFast(PIN_LEFT_DIR, HIGH); // break for at least 400us to un-sleep
      digitalWriteFast(PIN_LEFT_PWM, HIGH);
    }
    motorSleeping[0] = false;
    motorAwakeTime[0] = micros();
  }
  else
  {// disable motor - set to sleep
    if (not externalDriver)
    {
      pinMode(PIN_LEFT_DIR,OUTPUT);
      pinMode(PIN_LEFT_PWM,OUTPUT);
      digitalWriteFast(PIN_LEFT_DIR, LOW);
      digitalWriteFast(PIN_LEFT_PWM, LOW);
      motorSleeping[0] = true;
    }
  }
  if (motorEnable[1])
  { // out of sleep
    if (not externalDriver)
    {
      pinMode(PIN_RIGHT_DIR,OUTPUT);
      pinMode(PIN_RIGHT_PWM,OUTPUT);
      digitalWriteFast(PIN_RIGHT_DIR, HIGH);
      digitalWriteFast(PIN_RIGHT_PWM, HIGH);
    }
    motorSleeping[1] = false;
    motorAwakeTime[1] = micros();
  }
  else
  { // disable motor - set to sleep
    if (not externalDriver)
    {
      pinMode(PIN_RIGHT_DIR,OUTPUT);
      pinMode(PIN_RIGHT_PWM,OUTPUT);
      digitalWriteFast(PIN_RIGHT_DIR, LOW);
      digitalWriteFast(PIN_RIGHT_PWM, LOW);
      motorSleeping[1] = true;
    }
  }
  #else
  switch (robot.robotHWversion)
  {
    case 2:
    case 5:
      digitalWrite(M1DIS1, motorEnable[0]);
      digitalWrite(M2DIS1, motorEnable[1]);
      break;
    default:
      digitalWrite(M12DIS, motorEnable[0]);
      break;
  }
#endif
}


void UMotor::motorSetAnchorVoltage()
{
  float batteryNominalVoltage = robot.batteryVoltage;
  if (batteryNominalVoltage < 5.0)
    // not on battery - just for test and avoid divide by zero
    batteryNominalVoltage = 11.1;
  float scaleFactor = MAX_PWM / batteryNominalVoltage;
  // overload check
  if (overloadCount > 500 and motorEnable[0])
  { // disable motor (after 0.5 second)
    motorSetEnable(0, 0);
    //
    motorVoltage[0] = 0;
    motorVoltage[1] = 0;
    usb.send("# UMotor::motorSetAnchorVoltage: overload, disabled motors\n");
    //
  }
  // convert to PWM values (at 20khz)
  int w1, w2;
  if (motorReversed)
    // the big 12V Chinese motors
    scaleFactor *= -1.0;
  // else
  //   normal for pololu motors
  w1 = int16_t(motorVoltage[0] * scaleFactor);
  // the right motor must run the other way
  // debug - changed to no sign change for Lanciers robot
  if (robot.lanciers)
    // a bit strange that this robot don't like the minus sign (valid for ID 91 Frida)
    w2 = int16_t(motorVoltage[1] * scaleFactor);
  else
    w2 = int16_t(-motorVoltage[1] * scaleFactor);
    // debug end
  // implement
  motorSetPWM(w1, w2);
}

/** 
 * allowed input is +/- 2048, where 2048 is full battery voltage
 * */
void UMotor::motorSetPWM(int m1PWM, int m2PWM)
{ // make sure motor driver is in breake mode for at least 400us
  // this is important for HW9, blue PCB version 6.3
  if (robot.robotHWversion == 9 and (micros() - motorAwakeTime[0]) < 500)
  { // too soon to get out of break mode
    // usb.send("# UMotor::motorSetPWM: waiting for driver to go out of sleep\n");
    return;
  }
  // PWM is 12 bit
  const int max_pwm = int(MAX_PWM);
  // for logging
  if (m1PWM > 0)
    motorAnkerPWM[0] = m1PWM + pwmDeadband[0];
  else if (m1PWM < 0)
    motorAnkerPWM[0] = m1PWM - pwmDeadband[0];
  else
    motorAnkerPWM[0] = 0;
  if (m2PWM > 0)
    motorAnkerPWM[1] = m2PWM + pwmDeadband[1];
  else if (m2PWM < 0)
    motorAnkerPWM[1] = m2PWM - pwmDeadband[1];
  else
    motorAnkerPWM[1] = 0;
  motorAnkerDir[0] = m1PWM >= 0;
  motorAnkerDir[1] = m2PWM >= 0;
  
  if (false and tickCnt % 200 == 1)
  {
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# UMotor::motorSetPWM: changed m1:%d, m2:%d\n", m1PWM, m2PWM);
    usb.send(s);
  }
// #if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)

  if (not externalDriver)
  { // use on-board motor driver
    // LEFT motor
    if (m1PWM != 0 and motorSleeping[0])
    { // set to not sleeping by 1,1 for > 400us
      // one sample time should be sufficient
      // digitalWriteFast(PIN_LEFT_DIR, HIGH);
      // digitalWriteFast(PIN_LEFT_PWM, HIGH);
      // motorSleeping[0] = false;
      //     usb.send("# motor 1 out of sleep\n");
      usb.send("# motor 2 is sleeping, enable first\n");
    }
    if (not motorSleeping[0])
    {
      if (m1PWM > 0)
      {
        pinMode(PIN_LEFT_DIR,OUTPUT);
        digitalWriteFast(PIN_LEFT_DIR, HIGH);
        analogWrite(PIN_LEFT_PWM, max_pwm - motorAnkerPWM[0]);
      }
      else if (m1PWM < 0)
      { // move PWM to other pin
        pinMode(PIN_LEFT_PWM, OUTPUT); // motor 1 PWM
        digitalWriteFast(PIN_LEFT_PWM, HIGH);
        analogWrite(PIN_LEFT_DIR, max_pwm + motorAnkerPWM[0]);
      }
      else
      { // break
        pinMode(PIN_LEFT_DIR,OUTPUT);
        pinMode(PIN_LEFT_PWM,OUTPUT);
        digitalWriteFast(PIN_LEFT_DIR, HIGH);
        digitalWriteFast(PIN_LEFT_PWM, HIGH);
      }
    }
    // RIGHT motor
    if (m2PWM != 0 and motorSleeping[1])
    { // set to not sleeping by 1,1 for >400us
      // one sample time should be sufficient
      // digitalWriteFast(PIN_RIGHT_DIR, HIGH);
      // digitalWriteFast(PIN_RIGHT_PWM, HIGH);
      // motorSleeping[1] = false;
      usb.send("# motor 2 is sleeping, enable first\n");
    }
    if (not motorSleeping[1])
    {
      if (m2PWM > 0)
      {
        pinMode(PIN_RIGHT_DIR,OUTPUT);
        digitalWriteFast(PIN_RIGHT_DIR, HIGH);
        analogWrite(PIN_RIGHT_PWM, max_pwm - motorAnkerPWM[1]);
      }
      else if (m2PWM < 0)
      { // move PWM to other pin
        pinMode(PIN_RIGHT_PWM,OUTPUT);
        digitalWriteFast(PIN_RIGHT_PWM, HIGH);
        analogWrite(PIN_RIGHT_DIR, max_pwm + motorAnkerPWM[1]);
      }
      else
      { // break
        pinMode(PIN_RIGHT_DIR,OUTPUT);
        pinMode(PIN_RIGHT_PWM,OUTPUT);
        digitalWriteFast(PIN_RIGHT_DIR, HIGH);
        digitalWriteFast(PIN_RIGHT_PWM, HIGH);
      }
    }
  }
  else if (robot.robotHWversion == 9)
  { // PCB 6.3
    // the only with plug for external motor driver
    //
    int m1_PWM = PIN_LEFT_DIR;
    int m1_in1 = PIN_LEFT_PWM;
    int m1_in2 = PIN_LEFT_FAULT;
    setExternalDriverPWM(m1_PWM, m1_in1, m1_in2, m1PWM, &motorSleeping[0]);
    int m2_PWM = PIN_RIGHT_DIR;
    int m2_in1 = PIN_RIGHT_PWM;
    int m2_in2 = PIN_RIGHT_FAULT;
    setExternalDriverPWM(m2_PWM, m2_in1, m2_in2, m2PWM, &motorSleeping[1]);
  }
  else if (tickCnt % 100 == 0)
  {
    usb.send("# UMotor::motorSetPWM: no valid external motor driver or wrong HW type\r\n");
  }
//     else
// #else
//   // big motor controller only (not used anymore)
//   const int pwmOffsetFwd = 50; // compensation for slow rise time
//   const int pwmOffsetRev = 100; // motor controller around 12 V is a bit slower
//   int pwm1=-1, pwm2=-1, m1d=-1, m2d=-1;
//   switch (robot.robotHWversion)
//   {
//     case 1:
//     case 2:
//     case 3:
//     case 5:
//       // pololu 33926 dual controller (big controller with FB and overload detect)
//       // motor 1 (left)
//       if (m1PWM >= 0)
//       { // put H-bridge side 2 to high
//         m1d = HIGH;
//         if (m1PWM == 0)
//         { // make side 1 switch high
//           pwm1 = max_pwm;
//         }
//         else
//         { // make side 1 switch with low pulses down to fully low
//           pwm1 = max_pwm - m1PWM - pwmOffsetFwd;
//         }
//       }
//       else
//       { // put H-bridge side 2 to low
//         m1d = LOW;
//         // make side 1 switch with high pulses up to fully high
//         pwm1 = -m1PWM + pwmOffsetRev;
//       }
//       digitalWrite(M1DIR, m1d);
//       analogWrite(M1PWM, pwm1);
//       //
//       // motor 2 (rignt)
//       if (m2PWM >= 0)
//       { // put H-bridge side 2 to high
//         m2d = HIGH;
//         if (m2PWM == 0)
//         { // make side 1 switch high
//           pwm2 = max_pwm;
//         }
//         else
//         { // make side 1 switch with low pulses down to fully low
//           pwm2 = max_pwm - m2PWM - pwmOffsetFwd;
//         }
//       }
//       else
//       { // put H-bridge side 2 to low
//         m2d = LOW;
//         // make side 1 switch with high pulses up to fully high
//         pwm2 = -m2PWM + pwmOffsetRev;
//       }
//       digitalWrite(pinMotor2Dir, m2d);
//       analogWrite(pinMotor2Pwm, pwm2);
//       // debug
//       if (false and tickCnt % 100 == 0)
//       {
//         const int MSL = 150;
//         char s[MSL];
//         snprintf(s, MSL, "# UMotor::motorSetPWM PWM=%d %d, dir=%d %d, pwmpin=%d %d, motDir = %d %d\n", pwm1, pwm2, m1d, m2d, M1PWM, pinMotor2Pwm, M1DIR, pinMotor2Dir);
//         usb.send(s);
//       }
//       // debug end
//       break;
//       //
//     case 4:
//     case 6:
//       // pololu TB6612 dual motor controller (small controller)
//       if (m1PWM >= 0)
//       { // put H-bridge side 2 to high
//         digitalWrite(M1DIR, LOW);
//         // make side 1 switch with low pulses down to fully low
//         analogWrite(M1PWM, max_pwm - m1PWM);
//       }
//       else
//       { // put H-bridge side 2 to low
//         digitalWrite(M1DIR, HIGH);
//         // make side 1 switch with high pulses up to fully high
//         analogWrite(M1PWM, max_pwm + m1PWM);
//       }
//       if (m2PWM >= 0)
//       { // put H-bridge side 2 to high
//         digitalWrite(pinMotor2Dir, LOW);
//         // make side 1 switch with low pulses down to fully low
//         analogWrite(pinMotor2Pwm, max_pwm - m2PWM);
//       }
//       else
//       { // put H-bridge side 2 to low
//         digitalWrite(pinMotor2Dir, HIGH);
//         // make side 1 switch with high pulses up to fully high
//         analogWrite(pinMotor2Pwm, max_pwm + m2PWM);
//       }
//       break;
//     default:
//       break;
//   }
// #endif
}


void UMotor::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Motor -------\r\n");
  snprintf(reply, MRL, "# -- \tmotr V \tSet motor reversed gear; V=0 for small motors, V=1 for some big motors\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotv m1 m2 \tSet motor voltage -24.0..24.0 - and enable motors\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotfrq \tSet motor PWM frequency [100..100000], is %d\r\n", PWMfrq);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tdeadband L R\tSet PWM deadband ([0..100%%], is left=%.1f%%, right=%.1f%%\r\n", float(pwmDeadband[0])/MAX_PWM*100.0, float(pwmDeadband[1])/MAX_PWM*100.0);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \trelax A\tAllow relax motors when stationary, is=%d\r\n", allowRelax);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \txdrv T\tUse T:1 = external driver (DRI0042), 0=internal, is=%d\r\n", externalDriver);
  usb.send(reply);
}


bool UMotor::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "motr", 4) == 0)
  {
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    bool rev = strtol(p1, (char**)&p1, 10);
    if (p1 == &buf[4] or rev)
      motorReversed = true;
    else
      motorReversed = false;
  }
  else if (strncmp(buf, "motv", 4) == 0)
  {
    float m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtof(p1, (char**)&p1);
    m2 = strtof(p1, (char**)&p1);
    motorVoltage[0] = m1;
    motorVoltage[1] = m2;
    if ((fabs(m1) < 0.01) and (fabs(m2) < 0.01))
      motorSetEnable(0, 0);
    else
      motorSetEnable(1, 1);
    // debug
    // should be in tick() only
    motorSetAnchorVoltage();
//     usb.send("# setting motor voltage\n");
    // debug end 
  }
  else if (strncmp(buf, "motfrq ", 7) == 0)
  {
    const char * p1 = &buf[7];
    int frq = strtol(p1, nullptr, 10);
    if (frq < 100)
      frq = 100;
//     if (frq > 50000)
//       frq = 50000;
    setPWMfrq(frq);
  }
  else if (strncmp(buf, "deadband ", 9) == 0)
  {
    const char * p1 = &buf[9];
    char * p2, *p3;
    float deadbandLeft = strtof(p1, &p2);
    float deadbandRight = strtof(p2, &p3);
    if (p2 != p1)
      pwmDeadband[0] = int(deadbandLeft / 100.0 * MAX_PWM);
    if (p2 != p3)
      pwmDeadband[1] = int(deadbandRight / 100.0 * MAX_PWM);
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# PWM dead-band set to left=%.1f%%, right=%.1f%%\r\n", float(pwmDeadband[0])/MAX_PWM*100.0, float(pwmDeadband[1])/MAX_PWM*100.0);
    usb.send(s);
    usb.send("# NB! dead-band setting is not saved to flash (lost after reboot)\n");
  }
  else if (strncmp(buf, "relax ", 6) == 0)
  {
    const char * p1 = &buf[6];
    allowRelax = strtol(p1, nullptr, 10) != 0;
  }
  else if (strncmp(buf, "xdrv ", 5) == 0)
  {
    const char * p1 = &buf[5];
    externalDriver = strtol(p1, nullptr, 10) == 1;
  }
  else
    used = false;
  return used;
}

void UMotor::sendData(int item)
{
  if (item == 0)
    sendMotorValues();
  if (item == 1)
    sendMotorPWM();
}

void UMotor::sendMotorValues()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "mot %.2g %.2g %.3g %.4g %d %d\r\n",
           motorVoltage[0], motorVoltage[1], control.vel_ref[0], control.vel_ref[1], motorReversed, relax);
  usb.send(s);
}

void UMotor::sendMotorPWM()
{
  const int MSL = 150;
  char s[MSL];
// #ifdef REGBOT_HW41
//   int f1 = digitalReadFast(PIN_LEFT_FAULT);
//   int f2 = digitalReadFast(PIN_RIGHT_FAULT);
// #else
//   int f1=0;
//   int f2=0;
// #endif
  snprintf(s, MSL, "motpwm %d %d %d %d %d %d %d\r\n", 
           motorAnkerDir[0], motorAnkerPWM[0], motorAnkerDir[1], motorAnkerPWM[1], PWMfrq, m1ok, m2ok);
  usb.send(s);
}

void UMotor::setPWMfrq(const char* line)
{
  // pwmfrq 400\n
  const char * p1 = line;
  int frq = strtol(p1, (char**)&p1, 10);
  if (p1 != line)
    // valid number
    setPWMfrq(frq);
}

void UMotor::setPWMfrq(int frq)
{
  // pwmfrq 400\n
  if (frq > 150000) // limit'ish of motor driver
    PWMfrq=65000; // default
  else
    PWMfrq = frq;
  analogWriteFrequency(PIN_LEFT_DIR, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_LEFT_PWM, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_RIGHT_DIR, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_RIGHT_PWM, PWMfrq); /// frequency (Hz)
  // debug
  if (false)
  {
    const int MSL = 130;
    char s[MSL];
    snprintf(s, MSL, "# setting motor PWM frq to %d Hz\n", PWMfrq);
    usb.send(s);
  }
}

void UMotor::tick()
{ //
  tickCnt++;
  if (relax and allowRelax)
  { // break motor (to avoid rattling)
    motorVoltage[0] = 0;
    motorVoltage[1] = 0;
  }
  motorSetAnchorVoltage();
// #ifdef REGBOT_HW41
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  if (not externalDriver)
  {
    m1ok = digitalReadFast(PIN_LEFT_FAULT);
    m2ok = digitalReadFast(PIN_RIGHT_FAULT);
  }
#endif
}

///////////////////////////////////////////////////////

void UMotor::eePromSave()
{
  // save desired PWM FRQ
  uint16_t flags = 0;
  flags |= motorReversed << 0;
  flags |= encoder.velEstFast << 1;
  flags |= allowRelax << 2;
  flags |= externalDriver << 3;
  eeConfig.pushWord(flags);
  // save in kHz
  eeConfig.pushWord(PWMfrq/1000);
}

void UMotor::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    uint16_t flags = eeConfig.readWord();
    motorReversed = (flags & 0x01) > 0;
    encoder.velEstFast = (flags & 0x02) > 0;
    allowRelax = (flags & 0x04) > 0;
    externalDriver = (flags & 0x08) > 0;
    // stored value is in kHz
    PWMfrq = eeConfig.readWord() * 1000;
    // removing a default of 65000Hz, that
    // makes the motor controller too hot
    // due to a filter capacitor of ~100nF in the brushed motor.
//     if (PWMfrq > 30000)
//       PWMfrq = 19000;
    setup();
  }
  else
  { // skip robot specific items
    int skip = 2 + 2;
    eeConfig.skipAddr(skip);
  }
}

void UMotor::stopAllMotors()
{ // 
  motorVoltage[0] = 0;
  motorVoltage[1] = 0;
  motorPreEnabled = true;
  motorSetEnable(0,0);
}

void UMotor::setExternalDriverPWM(int pin_PWM, int pin_in1, int pin_in2, int pwm, bool * sleeping)
{
  // left motor
  if (not *sleeping)
  {
    if (pwm > 0)
    {
      digitalWriteFast(pin_in1, HIGH);
      digitalWriteFast(pin_in2, LOW);
      analogWrite(pin_PWM, pwm);
    }
    else if (pwm < 0)
    { // move PWM to other pin
      digitalWriteFast(pin_in1, LOW);
      digitalWriteFast(pin_in2, HIGH);
      analogWrite(pin_PWM, -pwm);
    }
    else
    { // break
      digitalWriteFast(pin_in1, LOW);
      digitalWriteFast(pin_in2, LOW);
      analogWrite(pin_PWM, 0);
    }
    if (false)
    { // debug
      const int MSL = 120;
      char s[MSL];
      snprintf(s, MSL, "# UMotor:: pin:pwm %d, in1 %d, in2 %d. PWM %d, relax %d\n", pin_PWM, pin_in1, pin_in2, pwm, relax);
      usb.send(s);
    }
  }
  else if (pwm != 0 and tickCnt % 10 == 0)
  { // set to not sleeping
    *sleeping = false;
    usb.send("# UMotor:: out of sleep\n");
  }
  else if (allowRelax)
  { // allow motor drivers to go into "vacant" mode
    *sleeping = true;
    digitalWriteFast(pin_in1, HIGH);
    digitalWriteFast(pin_in2, HIGH);
    usb.send("# UMotor:: into sleep\n");
  }
}
