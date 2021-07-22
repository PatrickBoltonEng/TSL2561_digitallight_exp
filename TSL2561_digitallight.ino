/*
 * Project MQ5
 * Description:  Seeed Studio MQ5 Gas Sensor
 * Author:  Patrick Bolton 
 * Date:  05/27/21
 */

#include "Particle.h"
#include "math.h"
#include "tsl2561.h"

SYSTEM_THREAD(ENABLED);

#define dbSerial Serial

TSL2561 tsl(TSL2561_ADDR_0);
		// TSL2561_ADDR_0 (0x29 address with '0', connected to GND)
		// TSL2561_ADDR   (0x39 default address, pin floating)
		// TSL2561_ADDR_1 (0x49 address with '1' connected to VIN)

// TSL sensor related vars
uint16_t tsl_integrationTime;
double tsl_illuminance;
uint32_t tsl_illuminance_int;
bool tsl_autoGainOn;
// TSL execution control var
bool tsl_operational;
// TSL status vars
char tsl_status[21] = "na";
char tsl_autoGain_s[4] = "na";
uint8_t tsl_error_code;
uint8_t tsl_gain_setting;

SerialLogHandler logHandler;

#define UPDATE_INTERVAL 5000  //1 sec = 1000 millis

unsigned long UpdateInterval;
int min_last, min_time;

void setup() 
{
  dbSerial.begin(9600);

  TSLsetup();

  //function on the cloud: change sensor exposure settings (mqx 4)
  Particle.function("setExposure", setExposure);

  UpdateInterval = millis();
  min_last=-1;
}

void loop()
{
  if(Particle.disconnected()){return;}

  if ((millis() - UpdateInterval) > UPDATE_INTERVAL)
  {
    getTSLdata();
    
    Log.info("TSL2561_Ill: %f", tsl_illuminance);

    min_time=Time.minute();
    if((min_time!=min_last)&&(min_time==0||min_time==15||min_time==30||min_time==45))
    {
      //createEventPayload1();
      min_last = min_time;    
      Log.info("Last Update: %d", min_last);
      Log.info(Time.timeStr());
    }
    UpdateInterval = millis();
  }
}


int setExposure(String command)
// cloud function to change exposure settings (gain and integration time)
//command is expected to be [gain={0,1,2},integrationTimeSwitch={0,1,2}]
// gain = 0:x1, 1: x16, 2: auto
// integrationTimeSwitch: 0: 14ms, 1: 101ms, 2:402ms
{
    // private vars
    char tsl_gainInput;
    uint8_t tsl_itSwitchInput;
    boolean tsl__setTimingReturn = false;

    // extract gain as char and integrationTime swithc as byte
    tsl_gainInput = command.charAt(0);//we expect 0, 1 or 2
    tsl_itSwitchInput = command.charAt(2) - '0';//we expect 0,1 or 2

    if (tsl_itSwitchInput >= 0 && tsl_itSwitchInput < 3){
      // acceptable integration time value, now check gain value
      if (tsl_gainInput=='0'){
        tsl__setTimingReturn = tsl.setTiming(false,tsl_itSwitchInput,tsl_integrationTime);
        tsl_autoGainOn = false;
      }
      else if (tsl_gainInput=='1') {
        tsl__setTimingReturn = tsl.setTiming(true,tsl_itSwitchInput,tsl_integrationTime);
        tsl_autoGainOn = false;
      }
      else if (tsl_gainInput=='2') {
        tsl_autoGainOn = true;
        // when auto gain is enabled, set starting gain to x16
        tsl__setTimingReturn = tsl.setTiming(true,tsl_itSwitchInput,tsl_integrationTime);
      }
      else{
        // no valid settings, raise error flag
        tsl__setTimingReturn = false;
      }
    }
    else{
      tsl__setTimingReturn = false;
    }

    // setTiming has an error
    if(!tsl__setTimingReturn){
        // set appropriate status variables
        tsl_error_code = tsl.getError();
        strcpy(tsl_status,"CloudSettingsError");
        //disable getting illuminance value
        tsl_operational = false;
        return -1;
    }
    else {
      // all is good
      tsl_operational = true;
      return 0;
    }
}

void TSLsetup()
{
  tsl_error_code = 0;
  tsl_operational = false;
  tsl_autoGainOn = false;

  //connecting to light sensor device
  if (tsl.begin()) {
    strcpy(tsl_status,"tsl2561 found");
  }
  else {
    strcpy(tsl_status,"tsl 2561 not found ");
  }

  // setting the sensor: gain x1 and 101ms integration time
  if(!tsl.setTiming(false,1,tsl_integrationTime))
  {
    tsl_error_code = tsl.getError();
    strcpy(tsl_status,"setTimingError");
    return;
  }

  if (!tsl.setPowerUp())
  {
    tsl_error_code = tsl.getError();
    strcpy(tsl_status,"PowerUPError");
    return;
  }

  // device initialized
  tsl_operational = true;
  strcpy(tsl_status,"initOK");
  return;
}

void getTSLdata()
{
  uint16_t tsl_broadband, tsl_ir;
  // update exposure settings display vars
  if (tsl._gain){tsl_gain_setting = 16;}
  else{tsl_gain_setting = 1;}

  if (tsl_autoGainOn){strcpy(tsl_autoGain_s,"yes");}
  else{strcpy(tsl_autoGain_s,"no");}

  if (tsl_operational)
  {
    // device operational, update status vars
    strcpy(tsl_status,"OK");
    // get raw data from sensor
    if(!tsl.getData(tsl_broadband,tsl_ir,tsl_autoGainOn))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"saturated?");
      tsl_operational = false;
    }
    // compute illuminance value in lux
    if(!tsl.getLux(tsl_integrationTime,tsl_broadband,tsl_ir,tsl_illuminance))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"getLuxError");
      tsl_operational = false;
    }
    // try the integer based calculation
    if(!tsl.getLuxInt(tsl_broadband,tsl_ir,tsl_illuminance_int))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"getLuxIntError");
      tsl_operational = false;
    }
  }
  else
  // device not set correctly
  {
    strcpy(tsl_status,"OperationError");
    tsl_illuminance = -1.0;
    // trying a fix
    // power down the sensor
    tsl.setPowerDown();
    delay(100);
    // re-init the sensor
    if (tsl.begin())
    {
      // power up
      tsl.setPowerUp();
      // re-configure
      tsl.setTiming(tsl._gain,1,tsl_integrationTime);
      // try to go back normal again
      tsl_operational = true;
    }
  }
}