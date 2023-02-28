#include "RET_StationManager.h"

void setup()
{
  uint16_t magic_number;
  char temp[4];
  int dat;

  ConfigureIO();
  Serial1.begin(9600);

  // turn on power supplies for testing
  Serial.printf("%s:SM 12V ON\r\n", header);
  digitalWriteFast(PS_12V, HIGH);
  delay(250);

  Serial.printf("%s:SM 24V ON\r\n", header);
  digitalWriteFast(PS_24V, HIGH);
  delay(250);

  Serial.printf("%s:SM 5V ON\r\n", header);
  digitalWriteFast(PS_5V, HIGH);
  delay(250);

  cmdCallBack.addCmd(strPowerSupply, &cmdPowerSupply);
  cmdCallBack.addCmd(strReset, &cmdReset);
  cmdCallBack.addCmd(strStreamTest, &cmdStreamTest);

  Serial.printf("%s:Initialize GPS\r\n", header);
  InitGPS();

  Serial.printf("%s:Pause for RF Supply Board power-on  ", header);
  for (int i = 5; i != 0; i--)
  {
    Serial.printf("%d ", i);
    delay(1000);
  }
  Serial.print("\r\n");

  int doloop = 1;
  while (doloop)
  {
    // check for communication to RF supply board
    temp[0] = 'V';
    temp[1] = 0;
    temp[2] = 0;
    temp[3] = 0;
    Serial1.write(temp, 4);
    dat = Response1();
    temp[0] = 0;
    Serial1.readBytes(temp, 2);
    if (temp[0] != 0 && temp[1] != 0)
    {
      Serial.printf("%s:RF Power Board V%c.%c detected\r\n", header, temp[0], temp[1]);
      doloop = 0;
    }
    else
      delay(250);
  }

  ResetRFPower();

  // configure high MPPT voltage cut off
  magic_number = 32 * 19; // approx 32V
  temp[0] = 'x';
  temp[1] = 0;
  temp[2] = (uint8_t)(magic_number & 0xff);
  temp[3] = (uint8_t)(magic_number >> 8);
  Serial1.write(temp, 4);
  dat = Response1();
  dat = dat;

  // configure low MPPT voltage cutoff
  magic_number = 21 * 19; // approx 21V
  temp[0] = 'n';
  temp[1] = 0;
  temp[2] = (uint8_t)(magic_number & 0xff);
  temp[3] = (uint8_t)(magic_number >> 8);
  // Serial.printf("write %02X %02X\r\n", temp[2], temp[3]);
  Serial1.write(temp, 4);
  dat = Response1();

  RFsuppliesON();

  Serial.printf("%s:RET Station Manager Initialized\r\n", header);
}
void loop()
{
  unsigned long currentTime = millis();

  cmdCallBack.updateCmdProcessing(&myParser, &myBuffer, &Serial);

  myGNSS.checkUblox();     // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (currentTime - previousTime >= eventTime)
  {
    digitalToggleFast(HEARTBEAT);
    if (++tic == 10)
    {
      // do the things
      RFQueryAll(0);
      CheckRFPowerInputFaults();
      CheckRFChannelFaults();
      ReportTemps();
      tic = 0;
    }
    previousTime = currentTime;
  }
}

void ConfigureIO(void)
{
  pinMode(PS_12V, OUTPUT);
  digitalWriteFast(PS_12V, LOW);
  pinMode(PS_24V, OUTPUT);
  digitalWriteFast(PS_24V, LOW);
  pinMode(PS_5V, OUTPUT);
  digitalWriteFast(PS_5V, LOW);
  pinMode(HEARTBEAT, OUTPUT);
  digitalWriteFast(HEARTBEAT, LOW);
  pinMode(FAN, OUTPUT);
  digitalWriteFast(FAN, LOW);
}

void cmdReset(CmdParser *myParser)
{
  ResetRFPower();
}

void cmdStreamTest(CmdParser *myParser)
{
}

void cmdPowerSupply(CmdParser *myParser)
{
  String s;
  char cmd; // channel
  char ch;  // channel
  uint16_t onoff;
  uint16_t pcount;

  pcount = myParser->getParamCount();
  if (pcount >= 2)
  {
    s = myParser->getCmdParam(1);
    s.toUpperCase();
    if (s.length() == 1)
      cmd = s.charAt(0);
    else
      cmd = 0;
    switch (cmd)
    {
    case 'R':
      ResetRFPower();
      break;
    case '?':
      RFQueryAll(1);
      break;
    case 'P':
      // change power state of a channel
      s = myParser->getCmdParam(2); // channel
      ch = s.toInt();
      s = myParser->getCmdParam(3); // state
      onoff = s.toInt();
      RFPower(ch, onoff, 1);
      break;
    case 'F':
      s = myParser->getCmdParam(2); // state
      onoff = s.toInt();
      Serial.printf("%s:fans %d\r\n", header, onoff);
      RFFans(onoff, 1);
      break;
    }
  }
}

void RFQueryAll(int verbose)
{
  uint8_t temp[4];
  int dat;

  // query all
  temp[0] = '?';
  temp[1] = 0;
  temp[2] = 0;
  temp[3] = 0;
  Flush1();
  Serial1.write(temp, 4);
  dat = Response1();
  if (verbose)
    Serial.printf("%s:%s\r\n", header, ((dat == 'O') ? "CMD OK" : "CMD BAD"));
  Serial1.readBytes((uint8_t *)&RFPowerSystemData, sizeof(SystemDataRead_t));
  if (verbose)
    Serial.printf("%s:\r\n"
                  "Board Temp : %00.2fC\n"
                  "Air Temp   : %00.2fC\n"
                  "MPPT V     : %00.2fV\n"
                  "Status     : %02X %02X\r\n",
                  header,
                  (RFPowerSystemData.board_temp * 0.0625),
                  (RFPowerSystemData.air_temp / 32.0 - 256.0),
                  (RFPowerSystemData.mppt_voltage * 0.0515625),
                  RFPowerSystemData.status,
                  RFPowerSystemData.air_temp_fault);
  for (int i = 0; i < 8; i++)
  {
    Serial1.readBytes((uint8_t *)&RFPowerCH[i], sizeof(PowerChannelSettingRead_t));
    if (verbose)
      Serial.printf("%s:CH %d : %00.2fV %0.1fA Status %02X %d\r\n", header, i + 1,
                    (RFPowerCH[i].voltage * 0.016),
                    (RFPowerCH[i].current * 0.001022),
                    RFPowerCH[i].status,
                    ExpectedRFPowerCHState[i]);
  }
}

void RFPower(int ch, int onoff, int verbose)
{
  uint8_t temp[4];
  Flush1();
  temp[0] = 'P';
  temp[1] = (uint8_t)ch;
  temp[2] = 0;
  temp[3] = (uint8_t)onoff;
  Serial1.write(temp, 4);
  int dat = Response1();
  if (verbose)
  {
    SendGPSTime();
    Serial.printf("%s:CH %d STATE %d ", header, ch, onoff);
    Serial.printf("%s:%s\r\n", header, ((dat == 'O') ? "CMD OK" : "CMD BAD"));
  }
  ExpectedRFPowerCHState[ch - 1] = onoff;
  // Serial.printf("ExpectedRFPowerCHState[%d] = %d", ch-1, ExpectedRFPowerCHState[ch-1]);
}

void RFFans(int st, int verbose)
{
  uint8_t temp[4];
  Flush1();
  temp[0] = 'F';
  temp[1] = 0;
  temp[2] = (uint8_t)st;
  temp[3] = 0;
  Serial1.write(temp, 4);
  int dat = Response1();
  if (verbose)
    Serial.printf("%s:%s\r\n", header, ((dat == 'O') ? "CMD OK" : "CMD BAD"));
}

void ResetRFPower(void)
{
  // all off
  char temp[] = {'P', 0, 0, 0};
  Serial1.write(temp, 4);
  Flush1();
  temp[0] = 'R'; // clear faults
  Serial1.write(temp, 4);
  Flush1();
  for (int i = 0; i < 8; i++)
    ExpectedRFPowerCHState[i] = 0;
}

void CheckRFPowerInputFaults(void)
{
  int any = 0;
  char msg[100];
  if ((RFPowerSystemData.status & MIN_VOLT_IN_FAULT) == MIN_VOLT_IN_FAULT)
  {
    sprintf(msg, "MINIMUM VOLTAGE INPUT FAULT, ATTEMPT RESET");
    any = 1;
  }
  if ((RFPowerSystemData.status & MAX_VOLT_IN_FAULT) == MAX_VOLT_IN_FAULT)
  {
    sprintf(msg, "MAXIMUM VOLTAGE INPUT FAULT, ATTEMPT RESET");
    any = 1;
  }
  if ((RFPowerSystemData.status & AIR_TEMP_FAULT) == AIR_TEMP_FAULT)
  {
    sprintf(msg, "AIR TEMP FAULT, ATTEMPT RESET");
    any = 1;
  }
  if (any == 1)
  {
    SendGPSTime();
    Serial.printf("%s:%s\r\n", header, msg);
    ResetRFPower();
    delay(100);
    RFsuppliesON();
  }
}

void CheckRFChannelFaults(void)
{
  int i;
  int any = 0;
  char msg[100];
  for (i = 0; i < 8; i++)
  {
#ifdef SKIP2
    if (i == 1)
      i++; // channel 2 (index 1) is bad, skip
#endif
    if ((RFPowerCH[i].status & CH_STATE) == 1)
    {
      if ((RFPowerCH[i].status & CH_CURRENT_MAX_FAULT) == CH_CURRENT_FAULT)
      {
        sprintf(msg, "CH %d CURRENT MAX FAULT, ATTEMPT RESET", i + 1);
        any = 1;
      }
      if ((RFPowerCH[i].status & CH_VOLT_MIN_FAULT) == CH_VOLT_MIN_FAULT)
      {
        sprintf(msg, "CH %d VOLTAGE OUT MIN FAULT, ATTEMPT RESET", i + 1);
        any = 1;
      }
      if ((RFPowerCH[i].status & CH_VOLT_MAX_FAULT) == CH_VOLT_MAX_FAULT)
      {
        sprintf(msg, "CH %d VOLTAGE OUT MAX FAULT, ATTEMPT RESET", i + 1);
        any = 1;
      }
      if (RFPowerCH[i].voltage < 625) // about 10 V
      {
        sprintf(msg, "CH %d VOLTAGE FAULT, ATTEMPT RESET %d", i + 1, RFPowerCH[i].voltage);
        any = 1;
      }
    }
  }
  for (i = 0; i < 8; i++)
  {
#ifdef SKIP2
    if (i == 1)
      i++; // channel 2 (index 1) is bad, skip
#endif
    if ((ExpectedRFPowerCHState[i] == 1) && ((RFPowerCH[i].status & CH_STATE) == 0))
    {
      sprintf(msg, "OUTPUT VOLTAGE(s) MISSING, ATTEMPT RESET");
      any = 1;
    }
  }
  if (any == 1)
  {
    SendGPSTime();
    Serial.printf("%s:%s\r\n", header, msg);
    ResetRFPower();
    delay(100);
    RFsuppliesON();
  }
}

void InitGPS(void)
{
  // int tries = 0;
  UBX_CFG_TP5_data_t timePulseParameters;
  if (GPS == 0)
    return;
  do
  {
    gnssSerial.begin(38400);
    if (myGNSS.begin(gnssSerial) == true)
      break;
    delay(100);
    gnssSerial.begin(9600);
    if (myGNSS.begin(gnssSerial) == true)
    {
      myGNSS.setSerialRate(38400);
      delay(100);
    }
    else
    {
      Serial.println("GNSS retry");
      delay(2000);
    }
  } while (1);

  myGNSS.setUART1Output(COM_TYPE_UBX); // turn off NEMA noise
  myGNSS.setI2COutput(COM_TYPE_UBX);   // turn off NEMA noise
  myGNSS.saveConfiguration();

  myGNSS.setNavigationFrequency(1); // one solution per second is good enough

  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
    Serial.printf("%s:Failed to retrieve time pulse parameters.\r\n", header);

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
  // While the module is _locking_ to GNSS time, make it generate 1Hz
  timePulseParameters.freqPeriod = SEC1;
  timePulseParameters.pulseLenRatio = 200 * MSEC1;
  // When the module is _locked_ to GNSS time, make it generate 1Hz
  // timePulseParameters.userConfigDelay = MSEC100 * 1000; // convert from us to ns please
  timePulseParameters.userConfigDelay = 0;
  timePulseParameters.freqPeriodLock = SEC1;
  timePulseParameters.pulseLenRatioLock = MSEC10;
  timePulseParameters.flags.bits.active = 1;         // Make sure the active flag is set to enable the time pulse.
  timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
  timePulseParameters.flags.bits.isFreq = 0;         // Tell the module that we want to set the period
  timePulseParameters.flags.bits.isLength = 1;       // Tell the module that pulseLenRatio is a length in us
  timePulseParameters.flags.bits.polarity = 1;       // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
    Serial.printf("%s:CH1 setTimePulseParameters failed!\r\n", header);
  else
    Serial.printf("%s:CH1 Success!\r\n", header);

  //  if (myGNSS.setAutoPVTcallbackPtr(&GetNAVdata) == true)
  //    Serial.println(F("setAutoPVTcallback successful"));
  //  else
  //    Serial.println(F("unable to link callback for PVT"));
  if (myGNSS.setAutoTIMTM2callbackPtr(&GetTIMdata) == true)
    Serial.println(F("setAutoPVTcallback successful"));
  else
    Serial.println(F("unable to link callback for PVT"));

  // attachInterrupt(digitalPinToInterrupt(GPS_T1_PIN), MyGPSISR, RISING);

  Serial.printf("%s:GNSS Init OK\r\n", header);
}

void SendGPSTime(void)
{
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  if (GPS == 0)
    return;

  year = myGNSS.getYear();
  month = myGNSS.getMonth();
  day = myGNSS.getDay();
  hour = myGNSS.getHour();
  minute = myGNSS.getMinute();
  second = myGNSS.getSecond();
  Serial.printf("%s:%02d-%02d-%02d %02d:%02d:%02d  ", header, year, month, day, hour, minute, second);



  myGNSS.getTIMTM2()
}

void GetNAVdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  uint16_t year = ubxDataStruct->year;
  uint8_t month = ubxDataStruct->month;
  uint8_t day = ubxDataStruct->day;
  uint8_t hour = ubxDataStruct->hour;
  uint8_t minute = ubxDataStruct->min;
  uint8_t second = ubxDataStruct->sec;
  Serial.printf("ZURF: EVENT %02d-%02d-%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, second);
}
void GetTIMdata(UBX_TIM_TM2_data_t *ubxDataStruct)
{
  double seconds;
  uint32_t ms;
  uint32_t ns;
  uint32_t week;
  time_t rawtime;
  struct tm *info;
  uint32_t fraction;

  week = ubxDataStruct->wnR;
  ms = ubxDataStruct->towMsR;
  ns = ubxDataStruct->towSubMsR;

  // convert to Unix Epoch seconds
  // 7*24*60*60=604800
  // 315964800 is difference between start of GPS and start of Unix Epoch
  seconds = ((double)week * 604800.0) + ((double)ms / 1000.0) + 315964800.0 + (ns / 1.0e6);
  rawtime = seconds;
  fraction = (uint32_t)((seconds - rawtime) * 1.0e6);
  info = gmtime(&rawtime); // fill out the Linux tm structure for easy reading
  // not sure if this is corrected for leap seconds?
  // if not, add 27 seconds
  // Serial.printf("%04d-%02d-%02d, %02d:%02d:%02d.%06d\r\n",
  //              1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday,
  //              -5 + info->tm_hour, info->tm_min, info->tm_sec, fraction);

  Serial.printf("ZURF: EVENT %02d-%02d-%02d %02d:%02d:%02d.%06d\r\n",
                1900 + info->tm_year,
                1 + info->tm_mon,
                info->tm_mday,
                info->tm_hour, info->tm_min, info->tm_sec, fraction);
}

void MyGPSISR(void)
{
  // does nothing
}

void ReportTemps(void)
{
  SendGPSTime();
  Serial.printf("Board %00.2f, Air %00.2f\r\n",
                (RFPowerSystemData.board_temp * 0.0625),
                (RFPowerSystemData.air_temp / 32.0 - 256.0));
}

void RFsuppliesON(void)
{
  Serial.printf("%s:FAN ON\r\n", header);
  RFFans(3, 1);
  delay(500);
  Serial.printf("%s:2nd 12V ON\r\n", header);
  RFPower(8, 1, 1);
  delay(500);
  Serial.printf("%s:RF Amplifier 24V #1 ON\r\n", header);
  RFPower(1, 1, 1);
  delay(500);
  Serial.printf("%s:RF Amplifier 24V #2 ON\r\n", header);
  RFPower(3, 1, 1);
  delay(500);
  Serial.printf("%s:RF Amplifier 24V #3 ON\r\n", header);
  RFPower(4, 1, 1);
}
