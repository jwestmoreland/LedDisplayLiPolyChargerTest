// *
// Example program to illustrate how to display time on an RGB display based on NMEA data received via GPS.
// Currently coded for PDT - you'll need to adjust for your timezone.
// 
// The source contained here goes with companion article for ARRL's QEX "A DIY NMEA Based GPS Time Display".
// Author - John C. Westmoreland
// Date -   April 27, 2020
// *
/* This example demonstrates how to include the ThreadMRI library which allows debugging of the Portenta H7 with GDB via a serial interface.

   To connect to the target, launch gdb with the following parameters

   arm-none-eabi-gdb -ex "set pagination off" --baud {230400} -ex "set target-charset ASCII" -ex "target remote {debug.port}" {project_name}.elf

   The baud rate needs to match the one provided in UartDebugCommInterface constructor, while {debug.port} depends on the operating system (eg. /dev/ttyUSB0 or COM15).

   If UsbDebugCommInterface is being used you can specify any baudrate.
*/

#define HCMS_DISPLAY_REV  1         // '2nd' rev of board

#include <Arduino.h>
// *
// Example program to illustrate how to display time on an RGB display based on NMEA data received via GPS.
// Currently coded for PDT - you'll need to adjust for your timezone.
// 
// The source contained here goes with companion article for ARRL's QEX "A DIY NMEA Based GPS Time Display".
// Author - John C. Westmoreland
// Date -   April 27, 2020
// *

// #define PORTENTA_H7 1       // define if using PORTENTA

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

// #include <ThreadDebug.h>
//
// UartDebugCommInterface debugComm(SERIAL1_TX, SERIAL1_RX, 230400);
// ThreadDebug            threadDebug(&debugComm, DEBUG_BREAK_IN_SETUP);

// UsbDebugCommInterface  debugComm(&SerialUSB);
// ThreadDebug            threadDebug(&debugComm, DEBUG_NO_BREAK_IN_SETUP);
// ThreadDebug            threadDebug(&debugComm, DEBUG_BREAK_IN_SETUP);

// #define PORTENTA_H7 1
// uncomment following if using the RGB Matrix Display From Arduino
// #define USING_ARDUINO_MATRIX 1

#ifndef PORTENTA_H7
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_LSM6DS3.h>
#include <LedDisplay.h>
#include <TemperatureZero.h>
#include <SAMD21turboPWM.h>

#else
#include <WiFi.h>                   // for portenta

#include <WiFiUdp.h>
#endif

#ifdef USING_ARDUINO_MATRIX
#include <SPI.h>
#include <ArduinoGraphics.h>
#include <Arduino_MKRRGB.h>
#endif

// #include "SECRET.H"

#define USE_IMU_CODE 1      // modify for other than Nano 33 IOT

// #define TEST_MODE 1       // If using the Hercules tool for debug
#define NO_TEST_MODE 1      // Running in normal mode using a GPS receiver (i.e. Themis' GPS via WiFi)

#define UTC_OFFSET  7       // this is PDT, so -8 from UTC 8 in Fall, 7 in Spring - currently 'manual'

#define USING_HCMS_DISPLAY 1      // if using HCMS Display

#define csPin 10      // SPI CS
#define i2cenPin 8   // low to use SPI
#define datardyPin 9

#if USING_HCMS_DISPLAY
#include "RM3100.h"
RM3100 rm3100;
#endif

// uncomment following if using the Lumex LDM-6432 RGB Display
// #define USING_LUMEX_DISPLAY 1
// #define USING_LATEST_FIRMWARE 1
// #define USING_LUMEX_DISPLAY 
// #define USING_LATEST_FIRMWARE 
// Note:  only one RGB display can be enabled at a time.

WiFiUDP _udp;

// static unsigned int dutyCycle = 0;
static unsigned char flip_flop = 0;

// REDIRECT_STDOUT_TO(Serial1);

#define SERIAL_COUNTER_TIME_OUT 5000    // so serial port doesn't hang the board
#define SERIAL_CTR_TIME_OUT 5
unsigned char utc_hr_buff[10];   // todo:  small buffer for UTC hour conversion
#if NO_TEST_MODE
#ifdef PORTENTA_H7
char* ssid     = SECRET_WIFI_NAME;
char* password = SECRET_PASSWORD;
#else
const char* ssid     = SECRET_WIFI_NAME;
const char* password = SECRET_PASSWORD;
#endif

// const char* host = "192.168.4.1";		// this is address of server
// IPAddress timeServer(192, 168, 4, 1);		// this is address of server
// const char* host = "192.168.4.1";    // this is address of server
// IPAddress timeServer(192, 168, 4, 1);   // this is address of server
const char* host = "239.192.1.2";    // this is address of PC w/UDP server
IPAddress timeServer(239, 192, 1, 2);   // this is address of PC w/UDP server
#endif
// to test
TurboPWM pwm;
#if TEST_MODE
#ifdef PORTENTA_H7
char* ssid     = SECRET_WIFI_NAME_TEST_MODE;
char* password = SECRET_PASSWORD_TEST_MODE;
#else
const char* ssid     = SECRET_WIFI_NAME_TEST_MODE;
const char* password = SECRET_PASSWORD_TEST_MODE;
#endif
/// const char* host = "192.168.2.249";		// this is address of PC w/UDP server
/// IPAddress timeServer(192, 168, 2, 249);		// this is address of PC w/UDP server
const char* host = "239.192.1.2";    // this is address of PC w/UDP server
IPAddress timeServer(239, 192, 1, 2);   // this is address of PC w/UDP server
#endif
unsigned int localPort = 123;      // local port to listen for UDP packets
const int httpPort = 123;
 
#if 0
String cloudSerialBuffer = ""; // the string used to compose network messages from the received characters
#endif

bool newData = false;

const int ledPin =  LED_BUILTIN;// the number of the LED pin
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 2000; // 1000;           // interval at which to blink (milliseconds)
// const long interval = 500;           // interval at which to blink (milliseconds)
static unsigned int loop_ctr = 0;

// char ldmCmd[] = "AT81=(0,0,TEST9876)";
// char ldmCmd1[] = "AT80=(0,0,A)";
char ldmCmdFMR[] = "AT20=()";
// char ldmCmdHdr[] = "AT81=(0,3,Time:)";
#ifndef USING_LATEST_FIRMWARE
char ldmCmdHdr[] = "ATe1=(0,3,96,Time:)";
#else
char ldmCmdHdr[] = "AT81=(0,3,Time:)";
#endif
char ldmBrightness[] = "ATf2=(10)";            // brightness
char ldmCmdMsg1[] = "ATe1=(2,2,3,Have A)";
// char ldmCmdMsg5[] = "ATe1=(2,0,3,GPS Actual)";
// char ldmCmdMsg5[] = "ATe1=(2,0,3,(GPS)Maker)";
#ifndef USING_LATEST_FIRMWARE
char ldmCmdMsg5[] = "ATe1=(2,0,3, WWVB Alt.)";
#else
// char ldmCmdMsg5[] = "AT81=(2,0, WWVB Alt.)";
char ldmCmdMsg5[] = "AT81=(2,0,GPS Atomic)";
#endif
char ldmCmdMsg3[] = "ATe1=(2,2,3,Happy )";
char ldmCmdMsg2[] = "ATe1=(3,1,3,Nice Day!)";
// char ldmCmdMsg6[] = "ATe1=(3,0,3,Real Time.)";
// char ldmCmdMsg6[] = "ATe1=(3,0,3,Faire 2019)";
#ifndef USING_LATEST_FIRMWARE
// char ldmCmdMsg6[] = "ATe1=(3,0,3, Time Sln.)";
char ldmCmdMsg6[] = "ATe1=(3,0,3,   Time)";
#else
// char ldmCmdMsg6[] = "AT81=(3,0, Time Sln.)";
char ldmCmdMsg6[] = "AT81=(3,0,   Time)";
#endif

char ldmCmdMsg4[] = "ATe1=(3,1,3,New Year!)";
// char ldmCmd[] = "AT81=(1,0,  HH:MM:SS)";                    // right now - just this format - char line x char column - 7 tall 5 wide
char ldmCmd1[] = "AT80=(0,0,A)";
#ifndef USING_LATEST_FIRMWARE
char ldmCmd[] = "ATe1=(1,0,111, HH:MM:SS)";
#else
char ldmCmd[] = "AT81=(1,0, HH:MM:SS)";
#endif

#ifdef USING_ARDUINO_MATRIX
char matrixString[] = "abc";					// can only display text clearly (now) with 4x6 font - only 3 chars at a time
#endif
#ifdef USING_ARDUINO_MATRIX
static void Write_Matrix_String(char *string);
#endif
static void Write_AT_Command(char *string);
int keyIndex = 0;
int value = 0;
// WiFiClient client;

// A UDP instance to let us send and receive packets over UDP

static char local_counter = 0;
#ifdef USING_HCMS_DISPLAY		// currently wired for Nano 33 IoT

// Define pins for the LED display.
// You can change these, just re-wire your board:
#define dataPin 2              // connects to the display's data in
#define registerSelect 3       // the display's register select pin
#define clockPin 4             // the display's clock pin
#define enable 5               // the display's chip enable pin
#if HCMS_DISPLAY_REV
// #define reset 6              // the display's reset pin
#define blank 6                 // the display's blank pin
#define reset 7              // the display's reset pin
// dutyCycle = 600;



#else
#define reset 6              // the display's reset pin


#endif

#define SHDN  14                    // A0
#define HIGH_CURRENT_MODE 15        // A1 XXX:::XXX relocate to digital pins



#define displayLength 8        // number of bytes needed to pad the string

// create an instance of the LED display:
#ifdef USING_HCMS_DISPLAY
#if HCMS_DISPLAY_REV
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin,
				  enable, reset, blank, displayLength);
#else
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin,
         enable, reset, displayLength);    
#endif             
#endif
int brightness = 15;        // screen brightness
/*
char myString[] = {
'p','r','i','n','t','i','n','g'};
*/
char myString[] = {
	'1','2',':','3','4',':','5','6'};  
TemperatureZero TempZero = TemperatureZero();
#endif
// #ifndef PORTENTA_H7

// #endif
  float t, Vin, Vbat;
  
// program setup
void setup()
{
  unsigned char i;
  unsigned int counter_main = 0;
  float x, y, z;
  
// debugBreak();
/// whd_print_logbuffer();
 pwm.setClockDivider(1, false); // Input clock is divided by 1 and sent to Generic Clock, Turbo is Off
 pwm.timer(0, 1, 600, false);   // Timer 2 is set to Generic Clock divided by 1, resolution is 600, phase-correct aka dual-slope PWM 
 
#ifndef PORTENTA_H7         // can't find <RingBuffer.h>
  setDebugMessageLevel(3); // used to set a level of granularity in information output [0...4]
#endif
Serial.begin(115200);
//  Serial1.begin(230400);
  delay(10);

#ifdef USING_ARDUINO_MATRIX
  MATRIX.begin();
  // set the brightness, supported values are 0 - 255
  MATRIX.brightness(10);
#endif

#if 1
 // Serial.println();
 // Serial.println();

  // in case serial port isn't connected so we don't hang the program:
  do {
    counter_main++;

  } while ( !Serial && ( counter_main < SERIAL_COUNTER_TIME_OUT) );
#endif

//  pinMode(LED_BUILTIN, OUTPUT);

// #define csPin 10      // SPI CS
// #define i2cenPin 9   // low to use SPI  

  

  /// Serial.println();
  /// Serial.println();

  // lumex display is connected to Serial1

  //  Serial1.flush();
  // Serial.println();
  // Serial.println()

// Serial1.begin(230400);
 Serial1.begin(115200);
 // Serial1.flush();
#if 0
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

counter_main = 0;
#if 1  // for Nano 33 IoT
// #if 1
// in case serial port isn't connected so we don't hang the program:
  do {
    counter_main++;

  } while ( !Serial1 && ( counter_main < SERIAL_COUNTER_TIME_OUT) );

  if ( (counter_main+1) >= SERIAL_COUNTER_TIME_OUT)
   Serial.println("Serial1 init timed out!\n");
// REDIRECT_STDOUT_TO(Serial);
// whd_print_logbuffer();
// whd_wifi_print_whd_log();
#endif

#ifdef USING_HCMS_DISPLAY
pinMode(SHDN, OUTPUT);          // SHDN tied low with weak pull-down
pinMode (HIGH_CURRENT_MODE, OUTPUT);  // gate of Q2 currently tied to A1 - N-Channel MOSFET so active high  xxx::: Linear Charger and LDO == HEAT
pinMode(A3, INPUT);

#if HCMS_DISPLAY_REV
// pinMode(blank, OUTPUT);
// pwm.analogWrite(blank, 400);
#endif
#endif

#ifdef USING_LATEST_FIRMWARE
  Write_AT_Command("ATef=(96)");
#endif
#ifdef USING_LUMEX_DISPLAY
  Write_AT_Command("ATd0=()");
  delay(100);
  Write_AT_Command("at20=()");
  delay(1000);
#endif
  // We start by connecting to a WiFi network

#if 0
  Serial1.println();
  Serial1.println();
  Serial1.print("Connecting to ");
  Serial1.println(ssid);
  delay(1000);
  #endif
///  WiFi.begin(ssid, password);
//  delay(10000);

#if 0
  while (WiFi.status() != WL_CONNECTED) {
    delay(10000);
    Serial1.print(".");                        // send to console if taking too long to connect to WiFi...
    WiFi.begin(ssid, password);
  }
#endif
#if 0
  Serial1.println("");
  Serial1.println("WiFi connected");
  Serial1.println("IP address: ");
  Serial1.println(WiFi.localIP());

 Serial1.println("\nStarting connection to server...");
 #endif
////  _udp.beginMulticast(IPAddress(239, 192, 1, 2), 123);           ///*** Using Multicast
//  Udp.begin(localPort);
  delay(100);

 Serial1.println("connected to server");

#ifdef USING_LUMEX_DISPLAY  
// #ifdef USING_HCMS_DISPLAY
  Write_AT_Command(ldmBrightness);
  delay(2);
// #endif

  Write_AT_Command("AT83=(0,0,EFGH9876)");          // just an optional test string output
  delay(1000);

  keyIndex = 0x30;
  // clear Lumex display
  Write_AT_Command("ATd0=()");
  delay(100);
  Write_AT_Command("AT27=(0)");
  delay(100);
  Write_AT_Command("AT29=(16,32,5,5,1)");
  delay(1000);
  Write_AT_Command(ldmCmdFMR);
  delay (2000);
  // Use WiFiClient class to create TCP connections
  //    WiFiClient client;

  Write_AT_Command("ATd0=()");
  delay(100);
#ifdef USING_LATEST_FIRMWARE
  Write_AT_Command("ATef=(96)");
#endif
  Write_AT_Command(ldmCmdHdr);
  delay(100);
#ifdef USING_LATEST_FIRMWARE
  Write_AT_Command("ATef=(111)");
#endif
  Write_AT_Command(ldmCmd);
  delay(100);
#ifdef USING_LATEST_FIRMWARE
  Write_AT_Command("ATef=(3)");
#endif
  Write_AT_Command(ldmCmdMsg5);
  delay(100);

  Write_AT_Command(ldmCmdMsg6);

#ifdef USING_LATEST_FIRMWARE
  Write_AT_Command("ATef=(111)");
#endif
#endif
#ifdef USING_ARDUINO_MATRIX
  MATRIX.beginDraw();
  MATRIX.noFill();
  MATRIX.noStroke();
  MATRIX.clear();
  MATRIX.stroke(0, 0, 255);
  MATRIX.textFont(Font_4x6);
  MATRIX.endDraw();
#endif

#ifdef USING_HCMS_DISPLAY 
 // initialize the display library:
  myDisplay.begin();
  myDisplay.setString("HH:MM:SS");
  myDisplay.home();
  myDisplay.setBrightness(brightness);
  for (int i = 0; i < 8; i++)
  {
	  myDisplay.setCursor(i);
	  myDisplay.write(myString[i]);
	  delay(3);
  }
  myDisplay.home();


  
#endif
#ifdef USING_HCMS_DISPLAY 
#if HCMS_DISPLAY_REV
// pinMode(blank, OUTPUT);
// digitalWrite(blank, LOW);
// pwm.analogWrite(blank, dutyCycle);
myDisplay.setBlankPin(4, 600);                // run PWM test on blank pin staring with 600 duty cycle
#endif
  delay(1000);

  // init utc hr buffer

  for ( i = 0; i < 9; i ++ ) {
	  utc_hr_buff[i] = 0x00;
  }

// for charger/HCMS Display board



digitalWrite(SHDN, HIGH);     // take charger out of shutdown
digitalWrite(HIGH_CURRENT_MODE, HIGH);     // will enable high-current mode     /// maybe check for battery first....
#endif

delay(1000);

#if USE_IMU_CODE
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

 //   while (1);
  } else {

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.println("\n\n");
 delay(100);
if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

   delay(100);
   Serial.println("\n\n");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
  
  Serial.println("\n\n");

 if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
    
  }
  
  Serial.print("Temperature sensor sample rate = ");
//  Serial.print(IMU.temperatureSampleRate());
//  Serial.println(" Hz");
  Serial.println();
  Serial.println("Temperature reading in degrees C");
  Serial.println("T");
    
  }

#endif
  
// set the digital pin as output:
 // pinMode(ledPin, OUTPUT); 
// #define csPin 10      // SPI CS
// #define i2cenPin 8   // low to use SPI  
// pinMode(csPin, OUTPUT); 
 pinMode(i2cenPin, OUTPUT); 
 pinMode(datardyPin, INPUT);
 attachInterrupt(digitalPinToInterrupt(datardyPin), dataRDYIRQ, RISING);

#if USE_IMU_CODE
 TempZero.init();
#endif

#ifdef USING_HCMS_DISPLAY
// analogOutputInit();
// t = readInternalTemperature();

  analogReference(AR_DEFAULT);   // should be 3.3V for this board

// change the resolution to 12 bits and read A0
  analogReadResolution(12);
  Serial.print(", 12-bit : ");
  Serial.print(analogRead(A3));
#endif
// whd_print_logbuffer();
// debugBreak();  
digitalWrite(i2cenPin, LOW);
delay(10);
SPI.begin();
// SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0));
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
// SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0));
    //Set sample rate
    rm3100.SetSampleRateReg(1); //in Hz
    rm3100.RunCMM(1);
    rm3100.SelfTest();
// SPI.endTransaction();
} // end setup

// this is the main even loop for the Arduino project
void loop()
{
//  float flat, flon;
  int temp;
  unsigned char ones, tens = 0;
  static unsigned long counter_temp_disp;
 

  char c, b, a = 0;
 static unsigned char counter = 0;
  static unsigned char arrayIndex = 0;
  static bool start_counter = false;
  static bool new_data = false;
  counter_temp_disp = 0;

  newData = false;
  counter = 0;
  unsigned long currentMillis = millis();

// simple, fast parser to decode incoming packet from _udp server to display
// todo:  check the checksum, currently, no checksum checking done.



 // if (_udp.parsePacket()) {
// debugBreak(); 
//// if (_udp.parsePacket())
{
 //    Serial.print(".");
////    while (_udp.available())
       
 // if (_udp.available())
  {
    //     while (client.available()) {
    //    c = client.read();
    c = _udp.read();             // character decode, we control what's being transmitted so no reason to have sophisticated parser here...
 #if 0 
    if (counter_temp_disp++ > 500) {
        Serial.print(c, HEX);
        counter_temp_disp = 0;
    }
#endif
 //   Serial.print(".");
   
    if ( c == 'Z' )

    {
    // Serial1.print(c, HEX);
      b = c;
    }
    if ( c == 'D' )
    {
    //  serial1.print(c, HEX);
      a = c;
    }
 
    if ( c == 'A' && b == 'Z' && a == 'D' )
    {
//        Serial1.print(c, HEX);
      if ( start_counter != true )
      {
        start_counter = true;
      }
      if ( new_data != true )
      {
        new_data = true;
      }
    }
    if ( start_counter )
    {
      counter++;

      switch ( counter )
      {
        case 3:
#ifndef USING_LATEST_FIRMWARE

          arrayIndex = 14;
          ldmCmd[arrayIndex] = 0x20;
          arrayIndex = 15;			// 15 16 - HR
          ldmCmd[arrayIndex] = c;               // correctiom here

#else
          arrayIndex = 10;
          ldmCmd[arrayIndex] = 0x20;
          arrayIndex = 11;      // 15 16 - HR
          ldmCmd[arrayIndex] = c;

#endif

          utc_hr_buff[7] = c;
          //                  tens = c - 0x30;
          tens = c;
          arrayIndex = 0;

#ifdef USING_ARDUINO_MATRIX

          matrixString[arrayIndex] = c;
#endif

#ifdef USING_HCMS_DISPLAY
	  
	  myString[arrayIndex] = c;

#endif	  
	  
          break;

        case 4:

#ifndef USING_LATEST_FIRMWARE

          arrayIndex = 16;			// 15 16 - HR
          ldmCmd[arrayIndex] = c;               // correction here
          arrayIndex = 1;
#else
          arrayIndex = 12;      // 15 16 - HR
          ldmCmd[arrayIndex] = c;               // correction here
          arrayIndex = 1;

#endif
#ifdef USING_ARDUINO_MATRIX
          matrixString[arrayIndex] = c;
#endif
#ifdef USING_HCMS_DISPLAY

	  myString[arrayIndex] = c;

#endif
	  
          // utc_hr_buff[8] = c;
          // utc_hr_buff[9] = 0x00;
          ones = c - 0x30;
          //                  ones = c - 0x37;

          switch ( tens )                     // needs to be fast and easily computed - maybe a LUT type of solution also...
          {
            case 0x30:

#ifndef USING_LATEST_FIRMWARE
              switch ( ldmCmd[16] )			// HR
#else
              switch ( ldmCmd[12] )      // HR
#endif
              {
                
// Spring - UTC - 7
// Fall - UTC - 8

                case 0x30:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x37;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x37;        // here:  0x36 or 0x37
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x37;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x37;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }
		  
#endif

		  
                  break;

                case 0x31:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x38;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x38;
#endif

#ifdef USING_ARDUINO_MATRIX

                  matrixString[0] = 0x31;
                  matrixString[1] = 0x38;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x38;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif

		  
                  break;

                case 0x32:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x39;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x39;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x39;
                  matrixString[2] = 0x3a;
                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }

#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x39;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
                  break;

                case 0x33:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x32;    // here
                  ldmCmd[16] = 0x30;
#else
                  ldmCmd[11] = 0x32;
                  ldmCmd[12] = 0x30;

#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x32;
                  matrixString[1] = 0x30;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x32;
		  myString[1] = 0x30;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
                  break;

                case 0x34:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x32;
                  ldmCmd[16] = 0x31;
#else
                  ldmCmd[11] = 0x32;
                  ldmCmd[12] = 0x31;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x32;
                  matrixString[1] = 0x31;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x32;
		  myString[1] = 0x31;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x35:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x32;
                  ldmCmd[16] = 0x32;
#else
                  ldmCmd[11] = 0x32;
                  ldmCmd[12] = 0x32;             // UTC - 8 - FALL
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x32;
                  matrixString[1] = 0x32;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x32;
		  myString[1] = 0x32;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x36:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x32;
                  ldmCmd[16] = 0x33;
#else
                  ldmCmd[11] = 0x32;
                  ldmCmd[12] = 0x33;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x32;
                  matrixString[1] = 0x33;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x32;
		  myString[1] = 0x33;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
		  break;
		  
                case 0x37:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x30;
#else
                  ldmCmd[11] = 0x30;      // fall - 0x32
                  ldmCmd[12] = 0x30;      // fall - 0x33
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x30;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x30;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x38:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x31;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x31;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x31;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x31;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif	  
                  break;

                case 0x39:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x32;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x32;

#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x32;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
		  
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x32;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                  default:
                  break;
              }
              break;

            case 0x31:
#ifndef USING_LATEST_FIRMWARE
              switch ( ldmCmd[16] )
#else
              switch ( ldmCmd[12] )
#endif
              {

                case 0x30:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x33;

#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x33;
#endif

#ifdef USING_ARDUINO_MATRIX

                  matrixString[0] = 0x30;
                  matrixString[1] = 0x33;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x33;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif

		  
                  break;

                case 0x31:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x34;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x34;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x34;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x34;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
		  
                  break;

                case 0x32:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x35;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x35;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x35;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x35;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif	  
                  break;

                case 0x33:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x36;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x36;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x36;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x36;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
                  break;

                case 0x34:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x37;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x37;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x37;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x37;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x35:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x38;

#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x38;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x38;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x38;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x36:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x30;
                  ldmCmd[16] = 0x39;
#else
                  ldmCmd[11] = 0x30;
                  ldmCmd[12] = 0x39;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x30;
                  matrixString[1] = 0x39;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x30;
		  myString[1] = 0x39;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif	  
                  break;

                case 0x37:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x30;
#else
                  ldmCmd[11] = 0x31;                  // TRANSITION POINT
                  ldmCmd[12] = 0x30;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x30;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x30;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x38:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x31;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x31;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x31;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x31;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x39:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x32;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x32;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x32;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x32;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif	  
                  break;

                default:
                  break;
              }
              break;

            case 0x32:
#ifndef USING_LATEST_FIRMWARE
              switch ( ldmCmd[16] )
#else
              switch ( ldmCmd[13] )      
#endif
              {

                case 0x30:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x33;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x33;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x33;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x33;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x31:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x34;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x34;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x34;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x34;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif		  
                  break;

                case 0x32:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x35;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x35;
#endif

#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x35;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x35;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif	  
                  break;

                case 0x33:
#ifndef USING_LATEST_FIRMWARE
                  ldmCmd[15] = 0x31;
                  ldmCmd[16] = 0x36;
#else
                  ldmCmd[11] = 0x31;
                  ldmCmd[12] = 0x36;
#endif


#ifdef USING_ARDUINO_MATRIX
                  matrixString[0] = 0x31;
                  matrixString[1] = 0x36;
                  matrixString[2] = 0x3a;

                  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
                  {
                    Write_Matrix_String(matrixString);
                    delay(250);
                  }
#endif
#ifdef USING_HCMS_DISPLAY
		  myString[0] = 0x31;
		  myString[1] = 0x36;
		  myString[2] = 0x3a;

		  if (ldmCmd[18] == 0x30 && ldmCmd[19] == 0x30)
		  {
			  myDisplay.setCursor(0);
			  myDisplay.write(myString[0]);
			  delay(1);
			  myDisplay.setCursor(1);
			  myDisplay.write(myString[1]);
			  delay(1);
			  myDisplay.setCursor(2);
			  myDisplay.write(myString[2]);
			  delay(1);
//			  Write_Matrix_String(matrixString);
//			  delay(250);
		  }

#endif
                  break;

                default:
                  break;

              }
              break;

            default:
              break;
          }

          break;

        case 5:

#ifndef USING_LATEST_FIRMWARE
          arrayIndex = 18;						// 18 19 - MIN
          ldmCmd[arrayIndex] = c;
#else
          arrayIndex = 14;						// 18 19 - MIN
          ldmCmd[arrayIndex] = c;
#endif

#ifdef USING_ARDUINO_MATRIX
          matrixString[0] = c;
#endif
#ifdef USING_HCMS_DISPLAY
	  myString[3] = c;
#endif  
          break;

        case 6:
#ifndef USING_LATEST_FIRMWARE
          arrayIndex = 19;						// 18 19 - MIN
          ldmCmd[arrayIndex] = c;
#else
          arrayIndex = 15;						// 18 19 - MIN
          ldmCmd[arrayIndex] = c;
#endif

#ifdef USING_ARDUINO_MATRIX
          matrixString[1] = c;
          matrixString[2] = 0x3a;

          if (ldmCmd[19] == 0x30)
          {
            Write_Matrix_String(matrixString);
            delay(250);
          }
#endif
#ifdef USING_HCMS_DISPLAY
	  myString[4] = c;
	  myString[5] = 0x3a;

	  if (ldmCmd[19] == 0x30)
	  {
		  myDisplay.setCursor(3);
		  myDisplay.write(myString[3]);
		  delay(1);
		  myDisplay.setCursor(4);
		  myDisplay.write(myString[4]);
		  delay(1);
		  myDisplay.setCursor(5);
		  myDisplay.write(myString[5]);
		  delay(1);		  
//		  Write_Matrix_String(matrixString);
//		  delay(250);
	  }
#endif
          break;

        case 7:
#ifndef USING_LATEST_FIRMWARE
          arrayIndex = 21;						// 21 22 - SEC
          ldmCmd[arrayIndex] = c;
#else
          arrayIndex = 17;						// 21 22 - SEC
          ldmCmd[arrayIndex] = c;

#endif

#ifdef USING_ARDUINO_MATRIX
          matrixString[2] = c;
          MATRIX.stroke(0, 0, 255);               // blue
          Write_Matrix_String(matrixString);
          delay(250);
#endif
#ifdef USING_HCMS_DISPLAY
	  myString[6] = c;
	  myDisplay.setCursor(6);
	  myDisplay.write(myString[6]);
//	  MATRIX.stroke(0, 0, 255);               // blue
//	  Write_Matrix_String(matrixString);
	  delay(1);
#endif
	  
          break;

        case 8:
#ifndef USING_LATEST_FIRMWARE
          arrayIndex = 22;						// 21 22 - SEC
          ldmCmd[arrayIndex] = c;
#else
          arrayIndex = 18;						// 21 22 - SEC
          ldmCmd[arrayIndex] = c;
#endif

#ifdef USING_ARDUINO_MATRIX
          if (ldmCmd[21] == 0x30 && ldmCmd[22] == 0x30)
          {
            Write_Matrix_String(matrixString);
            delay(250);
          }
          matrixString[2] = c;
#endif
#ifdef USING_HCMS_DISPLAY
	  myString[7] = c;
	  if (ldmCmd[21] == 0x30 && ldmCmd[22] == 0x30)
	  {
		  myDisplay.setCursor(6);
		  myDisplay.write(myString[6]);
		  delay(1);
		  myDisplay.setCursor(7);
		  myDisplay.write(myString[7]);
		  delay(1);
//		  myDisplay.write(myString[5]);
//		  delay(1);				  
//		  Write_Matrix_String(matrixString);
//		  delay(250);
	  }
//	  myString[7] = c;
#endif
	  
          start_counter = false;        // just doing seconds, not 1/10ths or 1/100rds... yet
          if ( new_data ) {
 #ifdef USING_LUMEX_DISPLAY

            Write_AT_Command(ldmCmd);
            //      MATRIX.stroke(0, 255, 0);               // green
 #endif
#ifdef USING_ARDUINO_MATRIX
            Write_Matrix_String(matrixString);
#endif

#if 1	    
#ifdef USING_HCMS_DISPLAY
//	    Write_Matrix_String(matrixString);
	    for (int i = 0; i < 8; i++)
	    {
		    myDisplay.setCursor(i);
		    myDisplay.write(myString[i]);
		    delay(1);
	    }
#if 0	    
	    brightness--;
	    
	    myDisplay.setBrightness(brightness);
	    if ( brightness < 5 )
		    brightness = 15;
#endif
	    
#endif
#endif	    
            //                  delay(1);
            new_data = false;
            counter = 0;
            a = 0x30;
	    b = 0x30;
#ifdef USING_HCMS_DISPLAY	    
	    myDisplay.home();
#endif	    
          }


          break;

        default:
#ifndef USING_LATEST_FIRMWARE
          arrayIndex = 14;
#else
          arrayIndex = 10;
#endif
          break;
      }

    }

  }

  }      // udp.parsepacket

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

#ifdef USING_HCMS_DISPLAY    

  float temperature = TempZero.readInternalTemperature();
#if 1
//  if ( flip_flop++ > 1 )
  {

//  pwm.analogWrite(blank, dutyCycle);  
#ifdef HCMS_REV_B
    myDisplay.setBlankPin(4, 600);          // continue with PWM test
#endif    
//  dutyCycle += 10;

//  flip_flop = 0;

  }


//  if ( dutyCycle > 1000 )
//      dutyCycle = 0;
#endif      
// Serial.print(dutyCycle);
// Serial.println( " %");
 
  // if there are incoming bytes available
  // from the server, read them and print them:

 Serial.print(temperature);
 Serial.println( " C");
 counter_temp_disp = 0;

  Serial.print(analogRead(A3));
  Serial.println(" :A3 Value");
  Vin = ((3.3 * analogRead(A3) )/ 4095 );
  Vbat = 1.275 * Vin;
  Serial.print(Vin);
  Serial.println(" :Vin"); 
  Serial.print(Vbat);
  Serial.println(" :Vbat");
#endif

#ifdef USING_HCMS_DISPLAY    
 if ( local_counter++ >= 20 ) {
 
  myDisplay.home();
  myDisplay.setCursor(0);
//  myDisplay.print("T:");
//  myDisplay.print(" ");
//  myDisplay.print(TempZero.readInternalTemperature(), DEC);
  myDisplay.print((TempZero.readInternalTemperature() * 1.8) + 32, DEC);
  myDisplay.setCursor(6);
  myDisplay.print(" F");
  delay(1000);
  local_counter = 0;
 }
  if ( loop_ctr++ >= 60 ) {
  // set the cursor first character
  myDisplay.home();
  myDisplay.setCursor(0);
//  myDisplay.print("A3: ");
  myDisplay.print(Vbat, DEC);
  myDisplay.setCursor(6);
  myDisplay.print("VB");
  delay(750);
  loop_ctr = 0;

  }

 
if (IMU.temperatureAvailable()) {
    // after IMU.readTemperature() returns, t will contain the temperature reading
    IMU.readTemperature(t);

    Serial.print(t);
    Serial.println(" C");

    
  }
#endif
 

   // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
//   digitalWrite(ledPin, ledState);
    delay(100);
// SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    //Set sample rate
    rm3100.SetSampleRateReg(1); //in Hz
                
    //Start to CMM run
    rm3100.RunCMM(1);
    //Clear Interrupt first
    rm3100.ClearDrdyInt();
       
 //   rm3100.DisplayCycleCount();
 rm3100.SetDrdyIntFlag(0);
    rm3100.ReadRM3100();
    rm3100.DisplayREVIDReg();
     rm3100.SelfTest();
// SPI.endTransaction();    
#if 0
// #define csPin 10      // SPI CS
// #define i2cenPin 9   // low to use SPI 
digitalWrite(i2cenPin, LOW);
delay(100);
digitalWrite(csPin, HIGH);
delay(100);
digitalWrite(csPin, LOW);
delay(100);
SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0));
SPI.transfer( (0xb4) );
temp = SPI.transfer( 0x00 );
SPI.endTransaction();
// digitalWrite(csPin, HIGH);
delay(50);
digitalWrite(i2cenPin, LOW);
delay(10);
Serial.println(temp,HEX);
#endif
    
  }
}  // end main loop()

#if 0
#ifdef USING_LUMEX_DISPLAY
// command to write string to Lumex LDM-6432 display
static void Write_AT_Command(char *string)
{

Serial1.print(string);
while (Serial1.read() != 'E') {;}
delay(2);

}
#endif
#endif
#if 1
// serial1 vs serial(0)!!!
#ifdef USING_LUMEX_DISPLAY
// command to write string to Lumex LDM-6432 display
static void Write_AT_Command(char *string)
{
static int counter_lum = 0;

 //    Serial1.flush();
 //  (Serial1.available()) {

     counter_lum = 0;

do {
    counter_lum++;
    delay(1);

  } while ( !(Serial1.available()) && ( counter_lum < SERIAL_CTR_TIME_OUT) );

counter_lum = 0;

 //   Serial1.write(string);
 //   Serial1.read();
    Serial1.print(string);
 // while (Serial1.read() != 'E') {

do {
    counter_lum++;
    delay(1);

  } while ( ( Serial1.read() != 'E') && ( counter_lum < SERIAL_CTR_TIME_OUT) );

 counter_lum = 0;

//     delay(1);
  
  delay(1);
 // }


}
#endif
#endif
#ifdef USING_ARDUINO_MATRIX
// command to write string to Arduino RGB Matrix
static void Write_Matrix_String(char *string)
{

#ifdef USING_ARDUINO_MATRIX

  MATRIX.beginDraw();
  //	MATRIX.clear();
  MATRIX.textFont(Font_4x6);
  //	MATRIX.stroke(0, 0, 255);

  MATRIX.text(string, 0, 1);
  MATRIX.endDraw();

  //  delay(2);
#endif
}
#endif

void dataRDYIRQ() {
   rm3100.SetDrdyIntFlag(1);
}



// *eof
