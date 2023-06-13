/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Erik-Home/Documents/GitHub/Photon2-Project_Erik/Sump-pump_Monitor/src/Sump-pump_Monitor.ino"
/*
 * Project Sump-pump_Monitor
 * Description: firmware for monitoring a sump pump basin
 * Author: Erik Fasnacht
 * Date: 6/12/23
 */

//library includes
#include "accurrent.h"
#include "temphum13.h"
#include "JsonParserGeneratorRK.h"

//#defines
//as

//enum for various status codes
void setup();
void loop();
void publish_status();
void ACcurrent_function();
uint8_t temphum13_function();
#line 17 "/Users/Erik-Home/Documents/GitHub/Photon2-Project_Erik/Sump-pump_Monitor/src/Sump-pump_Monitor.ino"
enum statusCodes
{
  PUMP_OFF = 0,
  PUMP_ON = 1,
  NORMAL_STATUS = 2,
  WATER_WARNING = 3,
  CURRENT_WARNING = 4,
  TEMPERATURE_WARNING = 5,
  HUMIDITY_WARNING = 6
};


// How often to check the sensor. Default: Every 1 second
const std::chrono::milliseconds currentInterval = 1s;           //check current sensor every 1 second
const std::chrono::milliseconds waterInterval = 10s;            //check water level every 10 second
const std::chrono::milliseconds temperatureInterval = 10min;    //check temp/hum sensor every 10 min
const std::chrono::milliseconds publishInterval = 60min;        //publish every 60 min

//particle system mode and thread
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_TRACE);   //set logging level

//static typedefs from libraries
static accurrent_t accurrent;
static temphum13_t temphum13;

//constant values
const float IDLE_CURRENT = 0.3;     //system idle current TODO, modify
const float HIGH_CURRENT = 15.0;    //high current event TODO, modify
const float LOW_TEMP = 45.0;        //low temp value
const float HIGH_TEMP = 95.0;       //high temp value    
const float HIGH_HUM = 80.0;        //high humidity value TODO, modify

//global variables
static float temperature;       //temperature
static float humidity;          //humdity
static uint8_t pumpStatus;      //pump status
static uint8_t systemStatus;    //system status
static uint16_t pumpCount;      //pump count


// setup() runs once, when the device is first turned on.
void setup() 
{
  //todo, add particle variables and functions
  
  Serial.begin(9600);   //set baud rate for debug messageds

  //todo, remove, only used for prototyping
  //Particle.disconnect();
  //WiFi.off();

  Particle.connect();

  //from temphum13 library
  temphum13_cfg_t temphum13_cfg;
  temphum13_cfg_setup( &temphum13_cfg );
  TEMPHUM13_MAP_MIKROBUS( temphum13_cfg, MIKROBUS_1 );    //set BUS1 for temp/hum click
  temphum13_init( &temphum13, &temphum13_cfg );   
  temphum13_default_cfg( &temphum13 );

  //from ac current library
  accurrent_cfg_t accurrent_cfg;
  accurrent_cfg_setup( &accurrent_cfg ); 
  ACCURRENT_MAP_MIKROBUS( accurrent_cfg, MIKROBUS_2 );    //set BUS2 for ac current click
  accurrent_init( &accurrent, &accurrent_cfg );

  
}

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{
  // The core of your code will likely live here.

  //variables for checking values intervals
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

  //check temperature/humidity
  if (millis() - tempCheck >= temperatureInterval.count())
	{
		tempCheck = millis();   //set current check to current time
    temphum13_function();   //get current temp/hum value
  }

  //check water level
  if (millis() - waterCheck >= waterInterval.count())
	{
		waterCheck = millis();   //set current check to current time
    //TODO insert water level function
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
        JsonWriterAutoObject obj(&jw);						                    //creates an object to pass JSON          
        jw.insertKeyValue("System Status", systemStatus);		          //set field for system status
        jw.insertKeyValue("Pump Status", pumpStatus);		              //set field for pump status
        jw.insertKeyValue("Pump Count over Last Hour", pumpCount);    //set field for pump count
        jw.insertKeyValue("Temperature (°F)", temperature);		        //set field for temperature
        jw.insertKeyValue("Humidity (%)", humidity);			            //set field for humidity
        jw.insertKeyValue("Time", Time.timeStr());			              //set field for time stamp
      }

      //Publish data packet consuming 1 of the pooled Cloud Data Operations (DOPs)
      Particle.publish("Status Message", jw.getBuffer(), PRIVATE );
    }
    
    pumpCount = 0;    //reset pump count
}

//accurrent function
void ACcurrent_function()
{
  //local variables
  float ac_current;             //variable for the measured current value
  static uint8_t temp_count;    //variable for the pump count

  //measure current
  ac_current = accurrent_get_a( &accurrent );             //measure the current
  Log.trace("Current value A : %06.4f \n", ac_current);   //debug message

  //pump cycle logic
  if (ac_current >= IDLE_CURRENT)    //value greater than idle threshold
  {
    if (temp_count == 0)    //pump just turned on
    {
      
      Log.trace("Pump just turned on");   //debug message
      pumpCount++;                        //increment the pump count
    }
    
    Log.trace("Pump is still on");    //debug message
    pumpStatus = PUMP_ON;
    temp_count++;                     //increment pump counter 
  }

  //high current event
  if (ac_current > HIGH_CURRENT)
  {
    Log.trace("High Current Event");   //debug message
    Particle.publish("High Current Event", String::format("%06.4f", ac_current));   //send warning message
  }

  //reset temp_count
  if((ac_current < IDLE_CURRENT) && (temp_count > 0))
  {
    temp_count = 0;
    pumpStatus = PUMP_OFF;
  }

}

//temphum13 function
uint8_t temphum13_function()
{
  //local variables
  static float tempC;
 // static
  
  tempC = temphum13_get_temperature( &temphum13 );
  humidity = temphum13_get_humidity( &temphum13 );
    
  if ((tempC != 65536.0) && (humidity != 65536.0))    //if values return don't equal 0xFF, IE I2C bus is idle
  {
    //set new value to global value
    temperature = (tempC * 1.8) + 32;		//convert celsuis to fahrenheit

    //debug messages
    Log.trace("Temperature °F : %.2f \n", temperature);
    Log.trace("Relative Humidity : %.2f \n", humidity);

    //temperature outside of window?
    if ((temperature > HIGH_TEMP) || (temperature < LOW_TEMP))
    {
      Log.trace("temperature outside of window");   //debug message
      Particle.publish("Temperature Warning", String::format("%.2f", temperature));   //send warning message
    }

    //humdity is too high?
    if (humidity > HIGH_HUM)
    {
      Log.trace("high humidity event");   //debug message
      Particle.publish("Humidity Warning", String::format("%.2f", humidity));   //send warning message
    }

    return TRUE;    //return success
  }

  return FALSE;   //return fail
}