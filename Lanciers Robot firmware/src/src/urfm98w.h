/***************************************************************************
 *   Copyright (C) 2025 by DTU
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


#pragma once
#include <unistd.h>
#include <SPI.h>

#include "usubss.h"
#include "RFM98W_library.h"

class URFM98w : public USubss
{
public:
  RFMLib * radio = nullptr; // =RFMLib(nss, DIO0, DIO5, RESET);
  /**
   * initialize */
  void setup();
  /**
   * Handle incomming messages */
  void tick();
  /**
   * decode servo related commands
   * \param buf is the command string
   * \returns true is used.
   * */
  bool decode(const char * buf) override;

  void sendHelp() override;
  /**
   * send subscribed data - called from subscription */
  void sendData(int item) override;
  /**
   * Receive finished interrupt */
  static void RFMISR();
  /**
   * original rx test code */
  void testRFM98w();
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();

  const char * versionh = "$Id: uservo.h 1698 2024-11-26 16:39:58Z jcan $";
  const char * versioncpp = nullptr;
private :
  /**
   * Received package */
  RFMLib::Packet package;
  bool newData = false;
  bool getData = false;
  /**
   * received string */
  static const int MAXRX = 256;
  /**
   * are radio to be used (not at the same time as line sensor) */
  static const int MAX_CONFIGS = 3;
  /*  # data sheet
   *  # register
   *  # byte 1: reg 1D, 4msb = BW 7 = 125 kHz, 9 = 500 kHz, 4 = 15.6 kHz
   *  #                 4lsb = error coding 2 = 4/5, 8=4/8
   *  # byte 2: reg 1E, 4msb spreading: 7=128 chips / symbol, 9=512 c/s, c=4096 c/s
   *  #                 4lsb mode: 4 = CRC on
   *  # byte 3: reg 09, bit 7=PAboost 1=used, 0=not, bit 6-4 = base power 0=10.8dBm,
   *                    4lsb power reduction 0=full power, 8 = half power f=minimum power (1dBm)
   *  # byte 4,5 not used.
   *  # byte 6: reg 26, 4msb = 0,
   *  #                 4lsb: 4=AGC internal, not mobile, C= AGC internal + mobile node
   */
  byte my_config[MAX_CONFIGS][6] = {{0x72,0x74,0x8F,0xAC,0xCD, 0x0C},  // fair - Pi ok
                                    {0x92,0x74,0x8F,0xAC,0xCD, 0x0C},  // more BW
                                    {0x92,0x78,0x8F,0xAC,0xCD, 0x0C}}; // more check
  int config = 0;
  bool use = false;
  unsigned int frequency = 434700000;
  int msgCnt = 0;
  int msgErrCnt = 0;
  /**
   * send radio status */
  void sendRfmStatus();
  /** set frequency (in Hz or MHz) */
  void setFrequency(double frq);
  /**
   * initiate radio - if not already setup */
  bool startRadio();
};

extern URFM98w rfm98w;

