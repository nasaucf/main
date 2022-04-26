/*
 * HTTP Client POST Request
 * Copyright (c) 2018, circuits4you.com
 * All rights reserved.
 * https://circuits4you.com 
 * Connects to WiFi HotSpot. */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <ArduCAM.h>
#include <Wire.h>
#include <SPI.h>
#include "memorysaver.h"
#if !(defined ESP8266 )
#error Please select the ArduCAM ESP8266 UNO board in the Tools/Board
#endif

//This demo can only work on OV2640_MINI_2MP or ARDUCAM_SHIELD_V2 platform.
#if !(defined (OV2640_MINI_2MP)||(defined (ARDUCAM_SHIELD_V2) && defined (OV2640_CAM)))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
// set GPIO16 as the slave select :
const int CS = 16;
//Version 1,set GPIO1 as the slave select :
const int SD_CS = 1;
ArduCAM myCAM(OV2640, CS);

String message = "";

bool messgaeReady = false;

/* Set these to your desired credentials. */
const char *ssid = "NETGEAR79";  //ENTER YOUR WIFI SETTINGS
const char *password = "widechair165";

//Web/Server address to read/write from 
String serverName = "http://137.184.54.94:4000/plantData/upload";   
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 15000;
float temp = 0, pressure = 0, humidity = 0, altitude = 0, vis = 0, ir = 0, uv = 0, TDS = 0;
float s415nm = 0, s445nm = 0, s480nm = 0, s515nm = 0, s555nm = 0, s590nm = 0, s630nm = 0, s680nm = 0, Clear = 0, NIR = 0;
String data = "";

//=======================================================================
//                    Power on setup
//=======================================================================


void setup() {
  Serial.begin(9600); 

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Timer set to 15 seconds (timerDelay variable), it will take 15 seconds before publishing the first reading.");
    uint8_t vid, pid;
  uint8_t temp;
  Wire.begin();
  Serial.println("ArduCAM Start!");

  //set the CS as an output:
  pinMode(CS,OUTPUT);

  //initialize SPI:
  SPI.begin();
  SPI.setFrequency(4000000); //4MHZ

  delay(1000);
  //Check if the ArduCAM SPI bus is OK
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55){
    Serial.println("SPI1 interface Error!");
    while(1);
  }
  


  
//Check if the camera module type is OV2640
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  if ((vid != 0x26 ) && (( !pid != 0x41 ) || ( pid != 0x42 )))
   Serial.println("Can't find OV2640 module!");
   else
   Serial.println("OV2640 detected.");
   myCAM.set_format(JPEG);
   myCAM.InitCAM();
   myCAM.OV2640_set_JPEG_size(OV2640_320x240);
}

void loop() {
  String img;
  String message = "";
  String everything ="";
  String imgmessage = "";
  // Send an HTTP POST request depending on timerDelay
  if ((millis() - lastTime) > timerDelay) {
    DynamicJsonDocument doc(4096);
    doc["type"] = "request";
    serializeJson(doc,Serial); 
    boolean messageReady =  false;
    while(messageReady == false) {
    if(Serial.available()) {
        message = Serial.readString();
        messageReady = true;
        }
      }
     DeserializationError error = deserializeJson(doc,message);
    if(error){
      Serial.print(F("seserializationJson failed: "));
      Serial.println(error.c_str());
      messageReady= false;
      return;
    }
    temp = doc["temp"];
    pressure = doc["pressure"];
    altitude = doc["altitude"];
    humidity = doc["humidity"];
    vis = doc["vis"];
    ir = doc["ir"];
    uv = doc["uv"];
    TDS = doc["TDS"];
    s445nm = doc["445nm"];
    s480nm = doc["480nm"];
    s515nm = doc["515nm"];
    s555nm = doc["555nm"];
    s590nm = doc["590nm"];
    s630nm = doc["630nm"];
    s680nm = doc["680nm"];
    Clear = doc["Clear"];
    NIR = doc["NIR"];
    /*
    String output = "temp:" + String(temp) + "\n";
    output += "pressure:" + String(pressure) + "\n";
    output += "altitude:" + String(altitude) + "\n";
    output += "humidity:" + String(humidity) +"\n";
    output += "vis:" + String(vis) + "\n";
    output += "ir:" + String(ir) +"\n";
    output += "uv:" + String(uv) + "\n";
    output += "TDS:" + String(TDS) ;
    String output2 = "s445nm:" + String(s445nm) + "\n";
    output2 += "s445nm:" + String(s445nm) + "\n";
    output2 += "s480nm:" + String(s480nm) + "\n";
    output2 += "s515nm:" + String(s515nm) +"\n";
    output2 += "s555nm:" + String(s555nm) + "\n";
    output2 += "s590nm:" + String(s590nm) +"\n";
    output2 += "s630nm:" + String(s630nm) + "\n";
    output2 += "s680nm:" + String(s680nm) +"\n";
    output2 += "Clear:" + String(Clear) + "\n";
    output2 += "NIR:" + String(NIR) +"\n";
    //Serial.println(output);
    //Serial.println(output2);


    //Serial.print("Printing camera data every 25 seconds");
    */
    Serial.println("HIT1");
    char str[8];
  byte buf[256];
  static int i = 0;
  static int k = 0;
  static int n = 0;
  uint8_t temp, temp_last;
 ;

  //Flush the FIFO
  myCAM.flush_fifo();
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();
  Serial.println("star Capture");
  while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
  Serial.println("Capture Done!");  

 
  i = 0;
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  #if !(defined (ARDUCAM_SHIELD_V2) && defined (OV2640_CAM))
  SPI.transfer(0xFF);
  #endif

 //Read JPEG data from FIFO
  while ( (temp !=0xD9) | (temp_last !=0xFF)){
  temp_last = temp;
  temp = SPI.transfer(0x00);
  //Write image data to buffer if not full
  if( i < 256)
   buf[i++] = temp;
   else{
    //Write 256 bytes image data to file
    myCAM.CS_HIGH();
    for (int x = 0; x < i; x++)
  {
    img += buf[x];
    img += ",";
  }


    i = 0;
    buf[i++] = temp;
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();
   }
   delay(0);  
 }
 
 //Write the remain bytes in the buffer
 if(i > 0){
  myCAM.CS_HIGH();
    for (int x = 0; x < i; x++)
  {
    img += buf[x];
    img += ",";
  }
 }
 //Close the file
  //Serial.println("CAM Save Done!");
  //Serial.println(img);
    Serial.println("HIT2");


  

    //Get sensor 
      //Get sensor data
    doc["temp"] = temp;
    doc["pressure"] = pressure;
    doc["altitude"] = altitude;
    doc["humidity"] = humidity;
    doc["vis"] = vis ;
    doc["ir"] = ir;
    doc["uv"] = uv;
    doc["TDS"] = TDS;    
    doc["445nm"] = s415nm;
    doc["480nm"] = s480nm;
    doc["515nm"] = s515nm;
    doc["555nm"] = s555nm;
    doc["590nm"] = s590nm;
    doc["630nm"] = s630nm;
    doc["680nm"] = s680nm;
    doc["Clear"] = Clear;
    doc["NIR"] = NIR;    
    doc["image"] = img;
    Serial.println("HIT3");
    serializeJson(doc,everything);
    Serial.println("HIT4");
    //Serial.print(everything);

    delay(500);
    

    delay(500);
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;
      http.begin(serverName);
      http.addHeader("Content-Type",  "application/json");  //Specify content-type header 
      int httpCode = http.POST(String(everything)); 
 
      //Send the request
      String payload = http.getString();                                        //Get the response payload
      Serial.println(httpCode);   //Print HTTP returMn code
      Serial.println(payload);    //Print request response payload
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

