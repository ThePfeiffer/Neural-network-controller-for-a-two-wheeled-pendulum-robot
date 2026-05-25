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

#include <ctype.h>
#include <unistd.h>
#include <SPI.h>
#include <string.h>

#include "uusb.h"
#include "RFM98W_library.h"
#include "urfm98w.h"
#include "urobot.h"
#include "ueeconfig.h"

URFM98w rfm98w;

// pins from Regbot version 6.3
#define NSS  26   // LS4  pin 26
#define DIO0 20   // LS0  pin 20
#define DIO1 22   // LS6  pin 22
#define DIO4 25   // LS7  pin 25
#define DIO5 27   // LS1  pin 27
#define RESET 21  // LS3  pin 21


void URFM98w::setup()
{
  SPI.begin();
  // Serial.begin(38400);
  //
  if (use)
    startRadio();
  //
  addPublistItem("rfm", "get RFM98w status 'rfm ok rxCnt errCnt config'");
  usb.addSubscriptionService(this);
  versioncpp = "$Id: urfm98w.cpp 1698 2024-11-26 16:39:58Z jcan $";
  //
}

bool URFM98w::startRadio()
{
  if (radio != nullptr)
  {
    radio->configure(my_config[config]);
    radio->setFrequency(frequency);
    //
    usb.send("# RFM98W Rx setup complete\n");
  }
  else
    usb.send("# RFM98W is not enabled\n");
  return true;
}


void URFM98w::tick()
{
  if (not use)
    return;
  if (radio == nullptr)
  {
    radio = new RFMLib(NSS, DIO0, DIO5, RESET);
    startRadio();
  }
  // radio active
  if (radio->rfm_status == 0)
  { // radio sleeping - wake up
    if (getData)
    {
      usb.send("# RFM get not implemented\n");
      getData = false;
    }
    else
    {
      radio->beginRX();
      attachInterrupt(DIO0, RFMISR, RISING);
    }
  }
  const int MSL = 300;
  const int MCL = 200;
  char s[MSL]; // error and support string
  char cmd[MCL]; // received command
  int m = 0; // string length
  // check for message
  if (radio->rfm_done)
  { // fetching data takes
    if (newData)
      usb.send("# URFM98w::tick: old received data is not read\n");
    // usb.send("# URFM98w::tick, got new package\n");
    radio->endRX(package);
    // convert to string
    int n = package.len;
    if (n > MCL - 1)
      n = MCL - 1;
    snprintf(s, MSL, "# first 4 = %d %d %d %d\n",
             package.data[0], package.data[1],
             package.data[2], package.data[3]);
    usb.send(s);
    for (int i = 4; i < n; i++)
    { // the first 4 bytes are ignored
      if (package.data[i] == '\n')
        break;
      if (package.data[i] >= ' ' and package.data[i] < 127)
      {
        cmd[m++] = package.data[i];
      }
      else
      {
        // snprintf(s, MSL, "# RFM98 package byte %d is %x (%d, %c)\n", i, package.data[i], package.data[i], (char)package.data[i]);
        // usb.send(s);
      }
    }
    cmd[m] = '\0';
    // CRC check
    if (package.crc)
    { // valid package
      newData = true;
    }
    else
    { // package with error
      snprintf(s, MSL, "# URFM98w::tick: CRC error data RSSI %d, '%s'\r\n", package.rssi, cmd);
      usb.send(s);
      newData = false;
    }
  }
  if (newData)
  { // make received data to a string
    package.data[package.len] = '\0';
    snprintf(s, MSL, "# URFM98w::tick: got %d chars, RSSI %d, crc=%d, '%s'\r\n", m, package.rssi, package.crc, cmd);
    usb.send(s);
    // check destination
    bool isValidCmd = true;
    const char * p1 = cmd;
    if (isdigit(*p1) and package.data[0] == 255)
    {
      //usb.send("# first char is number\n");
      int id = strtol(p1, (char**)&p1, 10);
      isValidCmd = id == robot.deviceID;
      // snprintf(s, MSL, "# id %d (my id %d) %d\n", id, robot.deviceID, (int)strlen(p1));
      // usb.send(s);
      while (*p1 == ' ')
        p1++;
      m = strlen(p1);
    }
    else if (package.data[0] < 255)
    {
      isValidCmd = package.data[0] == robot.deviceID;
    }
    if (isValidCmd)
    { // test also flags - not used, so should be 0
      isValidCmd = package.data[3] == 0;
    }
    isValidCmd &= islower(*p1);
    if (isValidCmd)
    { // check other characters too
      for (int i = 1; i < m; i++)
      {
        char c = p1[i];
        const char * valid = " .,:=+-\n\r";
        isValidCmd = (strchr(valid, c) != NULL) or islower(c) or isdigit(c);
        if (not isValidCmd)
          break;
      }
    }
    if (isValidCmd)
    { // the actual command must start with a lower case letter
      usb.send("# valid cmd: \n");
      usb.send(p1);
      command.parse_and_execute_command((char*)p1);
      msgCnt++;
    }
    else
    {
      snprintf(s, MSL, "# URFM98w: rejected '%s'\r\n", p1);
      usb.send(s);
      // usb.send("# RFM89W message is not a valid command\n");
      msgErrCnt++;
    }
    newData = false;
  }
}


void URFM98w::RFMISR()
{
  // Serial.print("RX interrupt");
  rfm98w.radio->rfm_done = true;
}



void URFM98w::testRFM98w()
{
  setup();
  if (radio->rfm_status == 0)
  {
    radio->beginRX();
    attachInterrupt(DIO0, RFMISR, RISING);
  }
  if (radio->rfm_done)
  {
    Serial.print(", got:");
    RFMLib::Packet rx;
    radio->endRX(rx);
    for(byte i = 0;i<rx.len;i++)
    {
      if (rx.data[i] < ' ')
      { // first 4 bytes are something else, but mostly low values
        Serial.print(rx.data[i]);
        Serial.print("  ");
      }
      else
      {
        Serial.print(char(rx.data[i]));
      }
    }
    // Serial.println();
    Serial.print(" crc ok:");
    Serial.print(rx.crc);
    Serial.print(" s/n:");
    Serial.print(rx.snr);
    Serial.print("dB, RSSI:");
    Serial.println(rx.rssi);
    rx.data[0] = '\0';
  }
}

bool URFM98w::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "rfmuse ", 7) == 0)
  { // new servo values
    const char * p1 = &buf[7];
    use = strtol(p1, nullptr, 10) == 1;
    if (use)
      startRadio();
  }
  else if (strncmp(buf, "rfmfrq ", 7) == 0)
  {
    const char * p1 = &buf[7];
    double frq = strtod(p1, nullptr);
    if (frq > 430000000 and frq < 440000000)
      frequency = (unsigned int)frq;
    else if (frq > 430 and frq < 440)
      frequency = (unsigned int)(frq * 1e6);
  }
  else if (strncmp(buf, "rfmcfg ", 7) == 0)
  {
    const char * p1 = &buf[7];
    int oldconfig = config;
    config = strtol(p1, nullptr, 10);
    if (config >= MAX_CONFIGS)
      config = MAX_CONFIGS - 1;
    else if (config < 0)
      config = 0;
    // NB requires reboot to implement a changed
    // I haven't found out if stop-rx before re-config or rfm-reset would work
    const int MSL = 180;
    char s[MSL];
    snprintf(s, MSL, "# RFM89W set to config %d (from %d)\n", config, oldconfig);
    usb.send(s);
  }
  else if (strncmp(buf, "rfmget", 6) == 0)
  {
    getData = true;
    usb.send("# get status over RFM\n");
  }
  else
    used = false;
  return used;
}

void URFM98w::setFrequency(double frq)
{
  if (frq > 430000000 and frq < 440000000)
    frequency = (unsigned int)frq;
  else if (frq > 430 and frq < 440)
    frequency = (unsigned int)(frq * 1e6);
  else
    usb.send("# RFM98W - not a legal 433MHz band frequency (not changed)\n");
}


void URFM98w::eePromLoad()
{
  if (eeConfig.isStringConfig())
  {
    eeConfig.skipAddr(1 + 4);
  }
  else
  {
    uint8_t flag = eeConfig.readByte();
    use = (flag & (1 << 0)) != 0;
    // next 3 bytes are config
    config = (flag >> 1) & 0x07;
    // offset
    int frq = eeConfig.read32();
    setFrequency(frq);
  }
}

void URFM98w::eePromSave()
{
  uint8_t flag = 0;
  // flags - no flags to save here
  if (use)
    flag +=  1 << 0;
  flag += (config & 0x07) << 1;
  eeConfig.pushByte(flag);
  eeConfig.push32(frequency); // not used anymore (steering servo)
}

void URFM98w::sendHelp()
{
  if (robot.robotHWversion >= 3 and robot.robotHWversion != 5)
  {
    const int MSL = 200;
    char s[MSL];
    usb.send("# RFM98W ------\n");
    usb.send("#    \tAll received messages for this robot (e.g.: '91 help') are send for decoding\n");
    snprintf(s, MSL, "# -- \trfmuse E \tE=1: Activate RFM98W radio (not to use with line sensor) is %d\r\n", use);
    usb.send(s);
    snprintf(s, MSL, "# -- \trfmfrq F \tSet radio frequency F= frequency in Hz (is %d)\r\n", frequency);
    usb.send(s);
    snprintf(s, MSL, "# -- \trfmcfg C \tSet radio mode configuration (0..2), is %d\r\n", config);
    usb.send(s);
  }
}


void URFM98w::sendData(int item)
{
  if (item == 0)
    sendRfmStatus();
}

void URFM98w::sendRfmStatus()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "rfm %d %d %d %d %d\n", use, frequency, msgCnt, msgErrCnt, config);
  usb.send(s);
}
