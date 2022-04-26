#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_AS7341.h>


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
float s415nm = 0, s445nm = 0, s480nm = 0, s515nm = 0, s555nm = 0, s590nm = 0, s630nm = 0, s680nm = 0, Clear = 0, NIR = 0;


Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_AS7341 as7341;

//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

String message = "";
bool messageReady = false; 

void setup() {
  Serial.begin(9600);
    while(!Serial);    // time to get serial running
    //Serial.println(F("BME280 test"));

    unsigned status;
    /*
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
     //   Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      //  Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      //  Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      //  Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      //  Serial.print("        ID of 0x60 represents a BME 280.\n");
      //  Serial.print("        ID of 0x61 represents a BME 680.\n");
      //  while (1) delay(10);
    }
    
    //Serial.println("-- Default Test --");
    delayTime = 1000;
/*
   // Serial.println();
     // Serial.println("Adafruit SI1145 test");
  
  if (! uv.begin()) {
  //  Serial.println("Didn't find Si1145");
    while (1);
  }

  //Serial.println("OK!");
 // Serial.println();

  pinMode(TdsSensorPin, INPUT);
  Serial.println();

  Serial.begin(9600);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }
  
  if (!as7341.begin()){
   //Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
  
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X); */
}

void loop() {
 // Serial.print("testloop");
 printValues();
    delay(delayTime);
  // put your main code here, to run repeatedly:
  uint16_t readings[12];
  float counts[12];

  if (!as7341.readAllChannels(readings)){
  //  Serial.println("Error reading all channels!");
    return;
  }

  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5) continue;
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    counts[i] = as7341.toBasicCounts(readings[i]);
  }

  s415nm = counts[0];
  s445nm = counts[1];
  s480nm = counts[2];
  s515nm = counts[3];
  s555nm = counts[6];
  s590nm = counts[7];
  s630nm = counts[8];
  s680nm = counts[9];
  Clear = counts[10];
  NIR = counts[11];

  while(Serial.available()){
    message = Serial.readString();
    messageReady = true;

  }
  if(messageReady){
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc,message);
    if(error){
      Serial.print(F("seserializationJson failed: "));
      Serial.println(error.c_str());
      messageReady= false;
      return;
    }
    if(doc["type"] == "request") {
      doc["type"] = "responce";
      //Get sensor data
      doc["temp"] = bme.readTemperature();
      doc["pressure"] = (bme.readPressure() / 100.0F);
      doc["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
      doc["humidity"] = bme.readHumidity();
      doc["vis"] = uv.readVisible();
      doc["ir"] = uv.readIR();
      doc["uv"] = (uv.readUV()/100);
      doc["TDS"] = tdsValue;    
      doc["445nm"] = s415nm;
      doc["480nm"] = s480nm;
      doc["515nm"] = s515nm;
      doc["555nm"] = s555nm;
      doc["590nm"] = s590nm;
      doc["630nm"] = s630nm;
      doc["680nm"] = s680nm;
      doc["Clear"] = Clear;
      doc["NIR"] = NIR;

      serializeJson(doc,Serial);
    }
    messageReady= false;
  }

}

void printValues() { /*
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
  delay(1000);

    Serial.print("Vis: "); Serial.println(uv.readVisible());
    Serial.print("IR: "); Serial.println(uv.readIR());
*/


  //float UVindex = uv.readUV();
  //UVindex /= 100.0;  
  //Serial.print("UV: ");  Serial.println(UVindex);

  //delay(1000);

    //Serial.println();  

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
   // Serial.print("TDS----Value:");
    //Serial.print(tdsValue, 0);
   // Serial.println("ppm");
    //Serial.println();
    //delay(1000);
  }
}
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
  
}

