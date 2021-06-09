// *
// Example program to illustrate how to display time on an RGB display based on NMEA data received via GPS.
// Currently coded for PDT - you'll need to adjust for your timezone.
// 
// The source contained here goes with companion article for ARRL's QEX "A DIY NMEA Based GPS Time Display".
// Author - John C. Westmoreland
// Date -   April 27, 2020
// *

#define PORTENTA_H7 1       // define if using PORTENTA

// this header is to put your AP information:

// the following are for normal operation from the AP the GPS receiver is attached to
#ifdef PORTENTA_H7
#define SECRET_WIFI_NAME (const char*)   "THEMIS_WS2"
#define SECRET_PASSWORD  (const char*)   "1234567890AB"
#else
#define SECRET_WIFI_NAME "THEMIS_WS2"
#define SECRET_PASSWORD  "1234567890AB"
#endif
// #define SECRET_WIFI_NAME (const char*)   "THEMIS_AT_WS"
// #define SECRET_PASSWORD  (const char*)   "1234567890"

// the following is for test purposes only
#define SECRET_WIFI_NAME_TEST_MODE (const char*)   "AJ6BC-ARGUS"
#define SECRET_PASSWORD_TEST_MODE  (const char*)   "4865D68D97D69C"
