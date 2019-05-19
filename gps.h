#ifndef _GPS_H
#define _GPS_H

#include <NMEAGPS.h>

void pollGpsOutput();
void setupGpsPort();
uint AutoBaud(HardwareSerial port);
void printGpsData();

extern NMEAGPS  gps;
extern gps_fix  fix;

#endif // _GPS_H
