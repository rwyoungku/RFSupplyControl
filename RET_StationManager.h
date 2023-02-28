#include <Arduino.h>
#include <stdio.h>
#include <time.h>
#include <Wire.h>
#include "Sparkfun_u-blox_GNSS_Arduino_Library.h"
#include "CmdBuffer.hpp"
#include "CmdCallback.hpp"
#include "CmdParser.hpp"

#ifndef __RET_StationManager__
#define __RET_StationManager__

#define GPS 1
#define SKIP2 1     /* channel 2 known to be bad on one board */

const char NAME[] = "RET Station Manager";
const char SN[] = "0001";
const char HW[] = "A";

const char header[] = "RETSM";

/*
 * magic numbers for times in units of us
 */
#define MSEC1 1000
#define MSEC10 10 * MSEC1
#define MSEC100 100 * MSEC1
#define MSEC250 250 * MSEC1
#define SEC1 1000 * MSEC1
#define SEC5 5 * SEC1
#define SEC10 10 * SEC1

#define PS_12V 2
#define PS_24V 3
#define PS_5V 41
#define HEARTBEAT 23
#define FAN 22

const unsigned long eventTime = 1000L;
unsigned long previousTime = 0L;
int tic = 0;

// GPS
SFE_UBLOX_GNSS myGNSS;
UBX_CFG_TP5_data_t timePulseParameters;
#define gnssSerial Serial8

// command parsing
CmdCallback<3> cmdCallBack; // # of commands
CmdBuffer<32> myBuffer;     // up to 32 bytes
CmdParser myParser;
const char strStreamTest[] = "STREAM";
const char strReset[] = "RESET";
const char strPowerSupply[] = "PS";
void cmdReset(CmdParser *myParser);
void cmdStreamTest(CmdParser *myParser);
void cmdPowerSupply(CmdParser *myParser);

void ConfigureIO(void);

struct SystemDataRead_s
{
    uint16_t board_temp;    // board temperature C = val*0.0625
    uint16_t air_temp;      // air temperature C = val/32-256
    uint16_t mppt_voltage;  // mppt/battery voltage V=val*0.0515625
    uint8_t status;         // bit 7 : unused
                            // bit 6 : fan mode 0=auto, 1=manual
                            // bit 5 : POR 0=OK, 1=manual
                            // bit 4 : air temp fault 0=no fault, 1=fault detected
                            // bit 3 : input volt min 0=no fault, 1=fault below minimum
                            // bit 2 : input volt max 0=no fault, 1=fault above maximum
                            // bit 1 : fan2 0=off, 1=on
                            // bit 0 : fan1 0=off, 1=on
    uint8_t air_temp_fault; // bit 7-5 : unused
                            // bit 4 : REFIN_HIGH
                            // bit 3 : REFIN_LOW
                            // bit 2 : RTDIN_LOW
                            // bit 1-0 : unused
} __attribute__((__packed__));
typedef SystemDataRead_s SystemDataRead_t;
#define AIR_TEMP_FAULT 0b00010000
#define MIN_VOLT_IN_FAULT 0b00001000
#define MAX_VOLT_IN_FAULT 0b00000100

typedef struct
{
    uint16_t volt_max;  // maximum allowed input voltage
                        // x = V / 0.0515625
    uint16_t volt_min;  // minimum allowed input voltage
                        // x = V / 0.0515625
    uint16_t temp_fan1; // turn on fan1
                        // x = (C+256) * 32
    uint16_t temp_fan2; // turn on fan2
                        // x = (C+256) * 32
} SystemDataWrite_t;

struct PowerChannelSettingRead_s
{
    uint16_t voltage; // output voltage V = x * 0.016
    uint16_t current; // output current I = x * 0.001022
    uint8_t status;   // bit 7-5 : unused
                      // bit 4 : current max 0=no fault, 1=fault
                      // bit 3 : volt min 0=no fault, 1=fault
                      // bit 2 : volt max 0=no fault, 1=fault
                      // bit 1 : current OK 1=OK, 0=fault
                      // bit 0 : channel state 0=off, 1=ON
} __attribute__((__packed__));
typedef PowerChannelSettingRead_s PowerChannelSettingRead_t;
#define CH_CURRENT_MAX_FAULT 0b00010000
#define CH_VOLT_MIN_FAULT 0b00001000
#define CH_VOLT_MAX_FAULT 0b00000100
#define CH_CURRENT_FAULT 0b00000010
#define CH_STATE 0b00000001

typedef struct
{
    uint16_t volt_max;    // max allowed voltage
    uint16_t volt_min;    // min allowed voltage
    uint16_t current_max; // max allowed current
} PowerChannelSettingWrite_t;

SystemDataRead_t RFPowerSystemData;
PowerChannelSettingRead_t RFPowerCH[8];
int ExpectedRFPowerCHState[8];

int ChannelsUsed[] = {1, 2, 4, 8, 0, 0, 0, 0};

uint16_t _bswap16(uint16_t a)
{
    a = ((a & 0x00FF) << 8) | ((a & 0xFF00) >> 8);
    return a;
}

void Flush1(void)
{
    while (Serial1.available())
        Serial1.read();
}

int Response1(void)
{
    int i = 50;
    while (!Serial1.available() && --i)
        delay(1);
    return Serial1.read();
}

void ResetRFPower(void);
void RFFans(int st, int verbose);
void RFPower(int ch, int onoff, int verbose);
void RFQueryAll(int verbose);
void CheckRFPowerInputFaults(void);
void CheckRFChannelFaults(void);
void ReportTemps(void);

void RFsuppliesON(void);

void InitGPS(void);
void SendGPSTime(void);
void GetNAVdata(UBX_NAV_PVT_data_t *ubxDataStruct);
void GetTIMdata(UBX_TIM_TM2_data_t  *ubxDataStruct);

void MyGPSISR(void);
#endif