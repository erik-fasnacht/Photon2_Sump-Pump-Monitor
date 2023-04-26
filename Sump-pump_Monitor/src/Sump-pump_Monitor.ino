/*
 * Project Sump-pump_Monitor
 * Description: firmware for monitoring a sump pump basin
 * Author: Erik Fasnacht
 * Date: 4/26/23
 */

//library includes
#include "accurrent.h"
#include "temphum13.h"
#include "JsonParserGeneratorRK.h"

//static tempdefs from libraries
static accurrent_t accurrent;
static temphum13_t temphum13;

//global variables
static float temperature;     //temperature variable
static float humidity;        //humdity variable
static uint8_t pumpStatus;    //pump status variable


// setup() runs once, when the device is first turned on.
void setup() 
{
  Serial.begin(9600);

  Particle.disconnect();
  WiFi.off();

  //from ac current
  accurrent_cfg_t accurrent_cfg;
  accurrent_cfg_setup( &accurrent_cfg ); 
  ACCURRENT_MAP_MIKROBUS( accurrent_cfg, MIKROBUS_1 );
  accurrent_init( &accurrent, &accurrent_cfg );

  //from temphum13
  temphum13_cfg_t temphum13_cfg;
  temphum13_cfg_setup( &temphum13_cfg );
  TEMPHUM13_MAP_MIKROBUS( temphum13_cfg, MIKROBUS_2 );
  temphum13_init( &temphum13, &temphum13_cfg );   
  temphum13_default_cfg( &temphum13 );
  
}

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{
  // The core of your code will likely live here.

}


void publish_status()
{
  //send publish only if cloud is connected
    if(Particle.connected() == TRUE)
    {
      //create JSON buffer and write values to it
      JsonWriterStatic<256> jw;		//creates a 256 byte buffer to write JSON to
      {
        JsonWriterAutoObject obj(&jw);						//creates an object to pass JSON          
        jw.insertKeyValue("Temperature (°F)", temperature);		//set field for temperature
        jw.insertKeyValue("Humidity (%)", humidity);			//set field for humidity
        jw.insertKeyValue("Time", Time.timeStr());			//set field for time stamp
      }

      //Publish data packet consuming 1 of the pooled Cloud Data Operations (DOPs)
      Particle.publish("System Status", jw.getBuffer(), PRIVATE );
    }
}

//accurrent function
void ACcurrent_function()
{
  float ac_current;
  ac_current = accurrent_get_ma( &accurrent );
  Serial.print("Current value = ");
	Serial.print(ac_current, DEC); 
	Serial.println("");
}

//temphum13 function
uint8_t temphum13_function()
{
  static float tempC;
  
  tempC = temphum13_get_temperature( &temphum13 );
  humidity = temphum13_get_humidity( &temphum13 );
    
  if ((tempC != 65536.0) && (humidity != 65536.0))    //if values return don't equal 0xFF, IE I2C bus is idle
  {
    //set new value to global value
    temperature = (tempC * 1.8) + 32;		//convert celsuis to fahrenheit

    //debug messages
    Serial.print("Temp in Celcuis = ");
	  Serial.print(tempC, DEC); 
	  Serial.println("");

    Serial.print("Temp in Fahrenheit = ");
	  Serial.print(temperature, DEC); 
	  Serial.println("");

    Serial.print("Relative Humidity = ");
	  Serial.print(humidity, DEC); 
	  Serial.println("");

    return TRUE;    //return success
  }

  return FALSE;   //return fail
}