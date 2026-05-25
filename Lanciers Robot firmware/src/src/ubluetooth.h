/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
 *   jca@elektro.dtu.dk            
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


#include "usubss.h"



class UBluetooth : public USubss
{
public:
  /**
   * setup */
  void setup();
  /**
   * decode data command */
  bool decode(const char * buf) override;
  /**
   * send help on messages */
  void sendHelp();
  /**
   * Send message to bluetooth module as is */
  void send(const char * msg, int from);
  /**
   * Do sensor processing - at tick time */
  void tick();  
  void tickRx();  /// check for incoming

  void eePromSave();

  void eePromLoad();
  
  const char * versionh = "$Id: ubluetooth.h 1740 2025-12-22 18:33:30Z jcan $";
  const char * versioncpp = nullptr;
  /**
   * 0 = disabled/not installed
   * 1 = module MC-10 BLE - no luck so far
   * 2 = module HC-42 Seems to work */
  int useBluetooth = 0;
protected:
  void setBaud(uint32_t baud);
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;
  void sendStatus();
  int outCnt{0};
  int inCnt{0};
  int tickCnt{0};
  int rxTickCnt{0};
  uint32_t baudrate = 9600;
  static const int MBRX = 100;
  char brx[MBRX]{0};
  int brxIdx = 0;
  char brxDecode[MBRX]{0};
  bool brxDecodeUsed = true;
  //
  friend class ULogger;
};

extern UBluetooth bt;

