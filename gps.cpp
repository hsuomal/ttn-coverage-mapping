#include <Arduino.h>
#include "globals.h"
#include "gps.h"


HardwareSerial gpsPort(1); // HardwareSerial gpsPort(Serial1); works as well

NMEAGPS  gps;
gps_fix  fix;

void printGpsData() {
    Serial.println("**********************");
    Serial.println("Talker    : "+(String)gps.talker_id[0]+(String)gps.talker_id[1]); // displays last talker
    Serial.print("Latitude  : ");
    Serial.println(fix.latitude(), 7);
    Serial.print("Longitude : ");
    Serial.println(fix.longitude(), 7);
    Serial.print("Altitude  : ");
    Serial.println(fix.altitude(),0);
    Serial.print("Satellites: ");
    Serial.println(gps.sat_count);
    Serial.print("HDOP      : ");
    Serial.println(fix.hdop*0.001 ,3);
    Serial.print("Time      : ");
    Serial.print(fix.dateTime.hours);
    Serial.print(":");
    Serial.print(fix.dateTime.minutes);
    Serial.print(":");
    Serial.println(fix.dateTime.seconds);
    Serial.println("**********************");
}

void pollGpsOutput() {
    while(gps.available( gpsPort )) // read all gps data there is
      fix = gps.read();
}

void setupGpsPort() {
    gpsPort.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX

    AutoBaud(gpsPort);
    gpsPort.setRxBufferSize(4196);  // Default buffer 256 bytes is pretty small
    
    //Serial.println("Setting baud to 57600");
    //gpsPort.print("$PUBX,41,1,0007,0003,57600,0*2B\r\n");
    //gpsPort.print("$PUBX,41,1,0007,0003,38400,0*20\r\n");
    //gpsPort.flush();
    //gpsPort.updateBaudRate(57600);
    Serial.println("Setting baud to 38400");
    gpsPort.print("$PUBX,41,1,0007,0003,38400,0*20\r\n");
    gpsPort.flush();
    gpsPort.updateBaudRate(38400);
}

// Detect baud by assuming there are messages beginning $GP
// If such message is found, assume we have a correct baud
//
// If we change baud from gps default, autobaud is required. After resetting cpu the gps is still sending at the
// different baud we set it before (because GPS module not reset), not default baud.

uint AutoBaud(HardwareSerial port) {
  uint32_t baud=4800; // Start at 4800
  bool     foundBaud=false;
  uint32_t millisecs;

  Serial.print("Trying baud");
  while (!foundBaud) {
    if (baud==4800) baud=9600;        // Select next baudratge to try
    else if(baud==9600) baud=19200;
    else if(baud==19200) baud=38400;
    else if(baud==38400) baud=57600;
    else if(baud==57600) baud=115200;
    else baud=9600;
    gpsPort.updateBaudRate(baud);


    Serial.print(", "+String(baud));
    char bytes[]={0,0,0};
    millisecs=millis();
    while(millis() < millisecs+1000 && !foundBaud) {
      if(gpsPort.available()) {
        bytes[0] = bytes[1]; bytes[1] = bytes[2]; bytes[2]=gpsPort.read();  // keep track of last three characters
      }
      if(bytes[0]=='$' && bytes[1]=='G' && bytes[2]=='P') // Look for "$GP" string
        foundBaud=true;
    }

  } // while (foundbaud==0)

  Serial.println(", Found baud "+String(baud));
  return baud;
}
