/*********************************************************************************
 *  MIT License
 *  
 *  Copyright (c) 2020-2022 Gregg E. Berman
 *  
 *  https://github.com/HomeSpan/HomeSpan
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 ********************************************************************************/
 
////////////////////////////////////////////////////////////
//                                                        //
//    HomeSpan: A HomeKit implementation for the ESP32    //
//    ------------------------------------------------    //
//                                                        //
//     Demonstrates how to use SpanPoint() to implement   //
//     two remote temperature sensors on separate ESP32   //
//     devices.                                           //
//                                                        //
//     This sketch is for the MAIN DEVICE that contains   //
//     all the usual HomeSpan logic, plus two instances   //
//     of SpanPoint to read temperatures from two other   //
//     remote devices.                                    //
//                                                        //
////////////////////////////////////////////////////////////
// #include <Adafruit_TestBed.h>
// extern Adafruit_TestBed TB;

#define DEFAULT_I2C_PORT &Wire

// Some boards have TWO I2C ports, how nifty. We should scan both
#if defined(ARDUINO_ARCH_RP2040) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) \
    || defined(ARDUINO_SAM_DUE) \
    || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define SECONDARY_I2C_PORT &Wire1
#endif

#include "HomeSpan.h"
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

struct RemoteTempSensor : Service::TemperatureSensor {

  SpanCharacteristic *temp;
  // SpanCharacteristic *hum;
  // SpanCharacteristic *fault;
  // SpanPoint *remoteTemp;
  // const char *name;
  // float temperature;
  // float humidity;
  Adafruit_SHT4x sht4;// = Adafruit_SHT4x();
  sensors_event_t s_hum, s_temp;
  
  RemoteTempSensor(Adafruit_SHT4x &sht) : Service::TemperatureSensor(){
    this->sht4 = sht;
  
    // this->name=name;
    
    temp=new Characteristic::CurrentTemperature(-10.0);      // set initial temperature
    temp->setRange(-50,150);                                 // expand temperature range to allow negative values
    
    // hum=new Characteristic::CurrentRelativeHumidity(40.0);      // set initial temperature
    // hum->setRange(0,100);                                 // expand temperature range to allow negative values
    // fault=new Characteristic::StatusFault(1);                // set initial state = fault

    // remoteTemp=new SpanPoint(macAddress,0,sizeof(float));    // create a SpanPoint with send size=0 and receive size=sizeof(float)

  } // end constructor

  void loop(){
    sht4.getEvent(&s_hum, &s_temp);
    temp->setVal(s_temp.temperature);
    // hum->setVal(s_hum.relative_humidity);    
  } // loop
  
};

struct RemoteHumiditySensor : Service::HumiditySensor {

  // SpanCharacteristic *temp;
  SpanCharacteristic *hum;
  // SpanCharacteristic *fault;
  // SpanPoint *remoteTemp;
  // const char *name;
  // float temperature;
  // float humidity;
  Adafruit_SHT4x sht4;// = Adafruit_SHT4x();
  sensors_event_t s_hum, s_temp;
  
  RemoteHumiditySensor(Adafruit_SHT4x &sht) : Service::HumiditySensor(){
    this->sht4 = sht;
  
    // this->name=name;
    
    // temp=new Characteristic::CurrentTemperature(-10.0);      // set initial temperature
    // temp->setRange(-50,150);                                 // expand temperature range to allow negative values
    
    hum=new Characteristic::CurrentRelativeHumidity(40.0);      // set initial temperature
    hum->setRange(0,100);                                 // expand temperature range to allow negative values
    // fault=new Characteristic::StatusFault(1);                // set initial state = fault

    // remoteTemp=new SpanPoint(macAddress,0,sizeof(float));    // create a SpanPoint with send size=0 and receive size=sizeof(float)

  } // end constructor

  void loop(){
    sht4.getEvent(&s_hum, &s_temp);
    // temp->setVal(s_temp.temperature);
    hum->setVal(s_hum.relative_humidity);    
  } // loop
  
};

//////////////////////////////////////

void setup() {
  
  Serial.begin(115200);

 

  homeSpan.setLogLevel(1);
  homeSpan.setStatusPixel(PIN_NEOPIXEL);
  homeSpan.begin(Category::Bridges,"Temp/Hum");

   Serial.println("so it begins");
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || \
      defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || \
      defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || \
      defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary ports must be manually
    // assigned their pins with setPins()!
    Serial.println("Setting up Wire1 for ESP32S2");
    Wire1.setPins(SDA1, SCL1);
  #endif
  Wire1.begin();

  Serial.println("Adafruit SHT4x test");
  while (! sht4.begin(&Wire1)) {
    Serial.println("Couldn't find SHT4x");
    delay(1000);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  
  new SpanAccessory();  
    new Service::AccessoryInformation();
      new Characteristic::Identify(); 
      
  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Name("Temperature");
      // new Characteristic::Name("Humidity");
    new RemoteTempSensor(sht4);        // pass MAC Address of Remote Device

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      // new Characteristic::Name("Temperature");
      new Characteristic::Name("Humidity");
    new RemoteHumiditySensor(sht4);        // pass MAC Address of Remote Device
} // end of setup()

//////////////////////////////////////

void loop(){
  
  homeSpan.poll();
  
} // end of loop()

//////////////////////////////////////
