/*
 * Project Sump-pump_Monitor
 * Description: firmware for monitoring a sump pump basin
 * Author: Erik Fasnacht
 * Date: 6/13/23
 */

//library includes
#include "accurrent.h"
#include "temphum13.h"
#include "HC_SR04.h"
#include "JsonParserGeneratorRK.h"

//#defines for application
#define WATERMESSAGE    5     //defines max number of messages for water level, limits publishes when error publishes occur
#define MAXMESSAGE      1     //defines max number of messages, limits publishes when error publishes occur
#define FLOATSTUCK      30    //defines number of seconds for pump to remain on until float stuck event occurs

//enum for various status codes
enum statusCodes
{
  NORMAL_STATUS       = 0,
  WATER_WARNING       = 1,
  CURRENT_WARNING     = 2,
  STUCK_WARNING       = 3,
  TEMPERATURE_WARNING = 4,
  HUMIDITY_WARNING    = 5,
  I2C_ERROR           = 6
};

//how often to check system, time intervals
const std::chrono::milliseconds currentInterval = 1s;           //check current sensor every 1 second
const std::chrono::milliseconds waterInterval = 10s;            //check water level every minute
const std::chrono::milliseconds temperatureInterval = 10min;    //check temp/hum sensor every 10 min
const std::chrono::milliseconds publishInterval = 60min;        //publish every 60 min

//define signals for hc-sr04 sensor
const int echoPin = A5;
const int trigPin = S4;
HC_SR04 rangefinder = HC_SR04(trigPin, echoPin);    //initializes pins for ultrasonic sensor

//particle system mode and thread
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);    //set logging level

//static typedefs from mikroE libraries
static accurrent_t accurrent;
static temphum13_t temphum13;

//constant values for sump pump basin
const float IDLE_CURRENT = 1.0;     //system idle current, if value is less than this, pump not on
const float HIGH_CURRENT = 10.0;    //high current event
const float HIGH_WATER = 8.0;       //in inches, less than this then secondary pump isn't working

//constant values for temperature/humidity
const float LOW_TEMP = 45.0;        //low temp value
const float HIGH_TEMP = 95.0;       //high temp value
const float HIGH_HUM = 80.0;        //high humidity value

//global variables
uint8_t systemStatus;     //system status
uint16_t pumpCount;       //pump count
float waterLevel;         //water level in inches
float ac_current;         //measured current value
float temperature;        //temperature
float humidity;           //humdity

//variables that get reset by particle function, limts amount of messages when warning/error 
uint16_t waterCount;      //count for water level message
uint16_t currentCount;    //count for high current message
uint16_t stuckCount;      //count for pump stuck on message
uint16_t tempCount;       //count for temperature message
uint16_t humCount;        //count for humidity message
uint16_t errorCount;      //count for sensor bus error


// setup() runs once, when the device is first turned on.
void setup() 
{
  //setup cloud functions, best practice to do first in setup loop
 	Particle.function("Reset Counts", resetCount);      //reset counts
  Particle.function("Stop Messages", stopMessage);    //set counts to above maximum count to stop messages
  Particle.function("Send Status", sendStatus);       //sends current status of system
  Particle.connect();                                 //connect to particle cloud

  Serial.begin(9600);             //set baud rate for debug messages
  systemStatus = NORMAL_STATUS;   //set system status, normal at startup

  //from temphum13 library
  temphum13_cfg_t temphum13_cfg;
  temphum13_cfg_setup(&temphum13_cfg);
  TEMPHUM13_MAP_MIKROBUS(temphum13_cfg, MIKROBUS_1);    //set BUS1 for temp/hum click
  temphum13_init(&temphum13, &temphum13_cfg);   
  temphum13_default_cfg(&temphum13);

  //from ac current library
  accurrent_cfg_t accurrent_cfg;
  accurrent_cfg_setup(&accurrent_cfg); 
  ACCURRENT_MAP_MIKROBUS(accurrent_cfg, MIKROBUS_2);    //set BUS2 for ac current click
  accurrent_init(&accurrent, &accurrent_cfg);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{
  //variables for checking value intervals
  static unsigned long currentCheck = 0;
	static unsigned long tempCheck = 0;
	static unsigned long waterCheck = 0;
  static unsigned long publishCheck = 0;

  //get current amperage value
	if (millis() - currentCheck >= currentInterval.count())
	{
		currentCheck = millis();    //set current check to current time
    ACcurrent_function();       //get current amperage value
  }

  //check water level
  if (millis() - waterCheck >= waterInterval.count())
	{
		waterCheck = millis();    //set current check to current time
    sr04_function();          //check water level   
  }

  //check temperature/humidity
  if (millis() - tempCheck >= temperatureInterval.count())
	{
		tempCheck = millis();   //set current check to current time
    temphum13_function();   //get current temp/hum value
  }

  //publish status message
  if (millis() - publishCheck >= publishInterval.count())
	{
		publishCheck = millis();    //set current check to current time
    publish_status();           //publish status message
  }
}

void publish_status()
{
  //send publish only if cloud is connected
  if(Particle.connected() == TRUE)
  {
    //create JSON buffer and write values to it
    JsonWriterStatic<256> jw;		//creates a 256 byte buffer to write JSON to
    {
      JsonWriterAutoObject obj(&jw);						          //creates an object to pass JSON          
      jw.insertKeyValue("System Status", systemStatus);   //set field for system status
      jw.insertKeyValue("Water Level", waterLevel);		    //set field for water level
      jw.insertKeyValue("Pump Count", pumpCount);         //set field for pump count
      jw.insertKeyValue("Temperature", temperature);		  //set field for temperature
      jw.insertKeyValue("Humidity", humidity);			      //set field for humidity
      jw.insertKeyValue("Time", Time.timeStr());			    //set field for time stamp
    }

    //Publish data packet consuming 1 of the pooled Cloud Data Operations (DOPs)
    Particle.publish("Status Message", jw.getBuffer(), PRIVATE );

    pumpCount = 0;    //reset pump count
  } 
  else
  {
    //store and forward
  } 
}

//reset countfunction
int resetCount (String value)
{  
  waterCount    = 0;    //count for water level message
  currentCount  = 0;    //count for high current message
  stuckCount    = 0;    //count for pump stuck on message
  tempCount     = 0;    //count for temperature message
  humCount      = 0;    //count for humidity message
  errorCount    = 0;    //count for sensor bus error

  return true;          //return true
}

//stop messages function
int stopMessage (String value)
{ 
  waterCount    = WATERMESSAGE;   //stops water level message
  currentCount  = MAXMESSAGE;     //stops high current message
  stuckCount    = MAXMESSAGE;     //stops pump stuck on message
  tempCount     = MAXMESSAGE;     //stops temperature message
  humCount      = MAXMESSAGE;     //stops humidity message
  errorCount    = MAXMESSAGE;     //stops sensor bus error

  return true;                    //return true
}

int sendStatus (String value)
{
  ACcurrent_function();   //pump function
  temphum13_function();   //temp/hum function
  sr04_function();        //ultrasonic sensor function

  publish_status();       //publish status message
  return true;            //return true
}

//accurrent function
void ACcurrent_function()
{
  //local variables
  static uint16_t loop_count;   //variable for determing if pump just turned on

  //measure current
  ac_current = accurrent_get_a( &accurrent );             //measure the current
  Log.trace("Current value A : %06.4f \n", ac_current);   //debug message

  //pump cycle logic
  if (ac_current >= IDLE_CURRENT)    //value greater than idle threshold
  {
    if (loop_count == 0)    //pump just turned on
    {
      Log.info("Pump just turned on");   //debug message
      pumpCount++;                        //increment the pump count
    }
    
    Log.info("Pump is still on");    //debug message
    loop_count++;                     //increment pump counter 
  }

  //pump stuck on event, float stuck up
  if (loop_count > FLOATSTUCK)
  {
    systemStatus = STUCK_WARNING;       //set system status
    Log.info("Float stuck Event");    //debug message
   
    if (stuckCount < MAXMESSAGE)
    {
      Particle.publish("Float stuck Event");    //send warning message, with current # of seconds pump has been stuck  
      publish_status();                         //publish status message   
    }

    stuckCount++;   //increment high current count
  }
  else    //normal operation
  {
    if ((systemStatus == STUCK_WARNING) && (systemStatus != CURRENT_WARNING))   //change status only if previous stuck float warning and high current warning isn't active
    {
      Log.info("Float stuck Event Reset");    //debug message
      systemStatus = NORMAL_STATUS;           //set system status
      stuckCount = 0;                         //reset count
      publish_status();                       //publish status message
    }
  }
  
  //high current event
  if (ac_current > HIGH_CURRENT)
  {
    systemStatus = CURRENT_WARNING;     //set system status
    Log.info("High Current Event");    //debug message

    if (currentCount < MAXMESSAGE)
    {     
      Particle.publish("High Current Event");   //send warning message
      publish_status();                         //publish status message
    }

    currentCount++;   //increment high current count
  }
  else    //normal operation
  {
    if ((systemStatus == CURRENT_WARNING) && (systemStatus != STUCK_WARNING))   //change status only if previous current warning and stuck float warning isn't active
    {
      Log.info("High Current Event Reset");   //debug message
      systemStatus = NORMAL_STATUS;           //set system status
      currentCount = 0;                       //reset count
      publish_status();                       //publish status message
    }
  }

  //reset loop coutner
  if((ac_current < IDLE_CURRENT) && (loop_count > 0))
  {
    loop_count = 0;   //reset loop counter
  }
}

//temphum13 function
void temphum13_function()
{
  //local variables
  float tempC;
  
  tempC = temphum13_get_temperature(&temphum13);    //get temperature from sensor
  
  if (tempC != 65536.0)    //if value return doesn't equal 0xFF, IE I2C bus is idle
  {
    //set new value to global value
    temperature = (tempC * 1.8) + 32;		              //convert celsuis to fahrenheit
    humidity = temphum13_get_humidity(&temphum13);    //get new humidity value

    if (systemStatus == I2C_ERROR)   //change status only if previous I2C error
    {
      Log.info("I2C Error Reset");   //debug message
      systemStatus = NORMAL_STATUS;   //set system status
      errorCount = 0;                 //reset error count (valid value read)
    }

    //debug messages
    Log.info("Temperature °F : %.2f \n", temperature);
    Log.info("Relative Humidity : %.2f \n", humidity);

    //temperature outside of window?
    if ((temperature > HIGH_TEMP) || (temperature < LOW_TEMP))
    {
      systemStatus = TEMPERATURE_WARNING;           //set system status
      Log.info("temperature outside of window");   //debug message

      if(tempCount < MAXMESSAGE)
      {
        Particle.publish("Temperature Warning");    //send warning message
        publish_status();                           //publish status message
      }

      tempCount++;    //increment message count
    }
    else    //normal operation
    {
      if ((systemStatus == TEMPERATURE_WARNING) && (systemStatus != HUMIDITY_WARNING))  //change status only if previous temperature warning and humidity warning isn't active
      {
        Log.info("Temperature Event Reset");    //debug message
        systemStatus = NORMAL_STATUS;           //set system status
        tempCount = 0;                          //reset count
        publish_status();                       //publish status message
      }
    }

    //humdity is too high?
    if (humidity > HIGH_HUM)
    {
      systemStatus = HUMIDITY_WARNING;    //set system status
      Log.info("high humidity event");   //debug message

      if(humCount < MAXMESSAGE)
      {
        Particle.publish("Humidity Warning");   //send warning message
        publish_status();                       //publish status message
      }

      humCount++;   //increment message count
    }
    else    //normal operation
    {
      if ((systemStatus == HUMIDITY_WARNING) && (systemStatus != TEMPERATURE_WARNING))    //change status only if previous humidity warning and temperature warning isn't active
      {
        Log.info("Humidity Event Reset");   //debug message
        systemStatus = NORMAL_STATUS;       //set system status
        humCount = 0;                       //reset count
        publish_status();                   //publish status message
      }  
    }
  }
  else
  {
    systemStatus = I2C_ERROR;    //set system status
    Log.info("I2C Bus Error");   //debug message
    
    if(errorCount < MAXMESSAGE)
    {
      Particle.publish("temp/humdity sensor error");   //send error message
    }

    errorCount++;   //increment error message count  
  }
}

//HC-SR04 function
void sr04_function()
{
  waterLevel = rangefinder.getDistanceInch();
  Log.info("Sensor in IN : %.2f \n", waterLevel);    //debug message

  if (waterLevel <= HIGH_WATER)
  {
    systemStatus = WATER_WARNING;     //set system status
    Log.info("high water event");    //debug message

    if(waterCount < WATERMESSAGE)
    {
      Particle.publish("Water Warning");    //send warning message
      publish_status();                     //publish status message
    }

    waterCount++;   //increment message count
  }
  else    //normal operation
  {
    if (systemStatus == WATER_WARNING)   //change status only if previous water level warning 
    {
      Log.info("Water Warning Reset");    //debug message
      systemStatus = NORMAL_STATUS;       //set system status
      waterCount = 0;                     //reset count
      publish_status();                   //publish status message
    }
  }
}
