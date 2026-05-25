/***************************************************************************
*   Copyright (C) 2019-2024 by DTU                             *
*   jcan@dtu.dk                                                *
*
*   Main function for small regulation control object (regbot)
*   build for Teensy 4.1,
*   intended for 31300/1 Linear control 1 (now 34721/34722)
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


//#include <cstdbool>
#include <malloc.h>
#include <IntervalTimer.h>
#include "src/main.h"
#include "src/ulog.h"
#include "src/umission.h"
#include "src/ulinesensor.h"
#include "src/ueeconfig.h"
#include "src/wifi8266.h"
#include "src/uservo.h"

#include "src/uusb.h"
#include "src/urobot.h"
#include "src/uencoder.h"
#include "src/umotor.h"
#include "src/umotortest.h"
#include "src/uad.h"
#include "src/ucurrent.h"
#include "src/uirdist.h"
#include "src/uimu2.h"
#include "src/udisplay.h"
#include "src/uusbhost.h"
#include "src/urfm98w.h"
#include "src/ubluetooth.h"
#include "src/uledband.h"
#include "src/uNN.h"



// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period - typically 1ms)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
volatile uint32_t tsec = 0; /// time that will not overrun
volatile uint32_t tusec = 0; /// time that will stay within [0...999999]
// float missionTime = 0; // time in seconds
// flag for start of new control period
volatile bool startNewCycle = false;
int pushHBlast = 0;
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///
const char * versioncpp = "$Id: regbot.ino 1775 2026-04-25 07:35:51Z jcan $";

// ////////////////////////////////////////


void setup()   // INITIALIZATION
{ 
  ad.setup();
  robot.setup();
  robot.setStatusLed(HIGH);
  usb.setup();
  command.setup();
  encoder.setup();
  ls.setup();
  irdist.setup();
  userMission.setup();
  control.setup();
  nn.setup();
  imu2.setup();
  usbhost.setup();
  bt.setup();
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, ( unsigned int ) 10 ); // heartbeat timer, value in usec
  // data logger init
  logger.setup();
  logger.setLogFlagDefault();
  logger.initLogStructure ( 100000 / robot.CONTROL_PERIOD_10us );
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.setup();
  // set configuration changes after setup load
  current.setup();
  servo.setup();  // set PWM for available servo pins
  motor.setup();  // set motor pins
  motortest.setup();
  display.setup();
  rfm98w.setup();
  ledband.setup();
  robot.setBatVoltageScale(); // for RIC
  // start heartbeat to USB
  robot.decode("sub hbt 400\n");
  robot.setStatusLed(LOW);

  // float input_73[NUMBER_OF_INPUTS] = {-0.05299711, -0.05310576, -0.05305143, -0.27800683, 0.03511090, 0.00865527};
  // float input_98[NUMBER_OF_INPUTS] = {-0.08060810, -0.08079379, -0.08070095, -0.27800683, 0.09182288, 0.01848194};
  // float input_206[NUMBER_OF_INPUTS] = {-0.14723121, -0.14723726, -0.14723423, -0.27800683, -0.02999328, 0.18832262};
  // float input_670[NUMBER_OF_INPUTS] = {-0.01799498, -0.01799497, -0.01799497, -0.01158881, 0.00170892, -0.00949536};
  // float input_806[NUMBER_OF_INPUTS] = {0.02734645, 0.02726752, 0.02730699, 0.26590786, -0.01275764, -0.00756208};
  // float input_927[NUMBER_OF_INPUTS] = {0.06104934, 0.06104441, 0.06104688, 0.26590786, -0.04565391, -0.14215771};
  // float input_1467[NUMBER_OF_INPUTS] = {-0.31187497, -0.31184083, -0.31185790, -0.05869078, -0.07133772, -0.10814629};
  // float input_1617[NUMBER_OF_INPUTS] = {0.15951328, 0.15926567, 0.15938948, 0.37993703, -0.13007826, -0.04022363};
  // float input_1739[NUMBER_OF_INPUTS] = {0.15541282, 0.15537671, 0.15539477, 0.37993703, 0.02013085, -0.29184687};
  // float input_2006[NUMBER_OF_INPUTS] = {-0.00179835, -0.00179807, -0.00179821, 0.00000000, -0.00777971, 0.05235190};
  
  // float* NNOut1 = nn.predict(input_73);
  // Serial.println("NN output: ");
  // Serial.println(NNOut1[0], 8);

  // float* NNOut2 = nn.predict(input_98);
  // Serial.println(NNOut2[0], 8);
  // float* NNOut3 = nn.predict(input_206);
  // Serial.println(NNOut3[0], 8);
  // float* NNOut4 = nn.predict(input_670);
  // Serial.println(NNOut4[0], 8);
  // float* NNOut5 = nn.predict(input_806);
  // Serial.println(NNOut5[0], 8);
  // float* NNOut6 = nn.predict(input_927);
  // Serial.println(NNOut6[0], 8);
  // float* NNOut7 = nn.predict(input_1467);
  // Serial.println(NNOut7[0], 8);
  // float* NNOut8 = nn.predict(input_1617);
  // Serial.println(NNOut8[0], 8);
  // float* NNOut9 = nn.predict(input_1739);
  // Serial.println(NNOut9[0], 8);
  // float* NNOut10 = nn.predict(input_2006);
  // Serial.println(NNOut10[0], 8);
  
  // userMission.clear();
  // userMission.addLine("vel=0.2, log = 1 : time=2"); // 1.8 er minimum med Batteri, 8 er max. Uden batteri, 4 er min og 5 er max, Hvorfor er vel= motorspænding.
  // userMission.addLine("vel=0 : time=1");
  // robot.missionStart = true;
}

int debugSaved = 0;
/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop ( void )
{
  control.resetControl();
  bool cycleStarted = false;
  robot.setStatusLed(LOW);
  // - listen for incoming commands
  //   and at regular intervals (1ms)
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  while ( true ) 
  { // main loop
    usb.tick(); // service commands from USB
    bt.tickRx(); // optional Bluetooth module
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle ) // start of new control cycle
    { // error detect
      startNewCycle = false;
      cycleStarted = true;
      // AD converter should start as soon as possible, to also get a reading at half time
      // values are not assumed to change faster than this
      ad.tick();
      // robot.timing is to get some statistics on which part uses the CPU time
      robot.timing(1);
      // estimate velocity and pose
      encoder.tick();
      // calculate motor current
      current.tick();
      // net new acc/gyro measurements
      imu2.tick();
      // record read sensor time
      robot.timing(2);
      // calculate sensor-related values
      // process line sensor readings and
      // estimate line edge posiitons
      ls.tick();
      // distance sensor (sharp sensor)
      irdist.tick();
      // advance mission
      userMission.tick();
      // do control
      control.tick();
      // Implement on actuators
      servo.tick();
      motor.tick();
      // optional, summarize for motor parameter estimate
      motortest.tick();
      // monitor robot state
      robot.tick();
      // record read sensor time + control time
      robot.timing(3);
      // non-critical functions
      // save selected log data to RAM buffer
      logger.tick();
      // update display
      display.tick();
      // update ledband, if mounted
      ledband.tick();
      // get commands from LoRa radio
      rfm98w.tick();
      // optional module bluetooth in/out service
      bt.tick();
      // service USB-host plug
      usbhost.tick();
    }
    // loop end time
    if (cycleStarted)
    { // mostly timing (total sample timing)
      robot.timing(4);
      robot.saveCycleTime();
      cycleStarted = false;
    }
  }
}

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{ // as basis for all timing
  hb10us++;
  tusec += 10;
  if (tusec > 1000000)
  {
    tsec++;
    tusec = 0;
  }
  if ( hb10us % robot.CONTROL_PERIOD_10us == 0 ) // main control period start
  { // time to start new processing sample
    userMission.missionTime += 1e-5 * robot.CONTROL_PERIOD_10us;
    hbTimerCnt++;
    startNewCycle = true;
    robot.timing(0);
  }
  if ( int(hb10us % robot.CONTROL_PERIOD_10us) == robot.CONTROL_PERIOD_10us/2 ) // start half-time ad conversion
  { // Time to read a LEDs off value (and turn LEDs on for next sample)
    ad.tickHalfTime();
  }
}

/////////////////////////////////////////////////////////////////

