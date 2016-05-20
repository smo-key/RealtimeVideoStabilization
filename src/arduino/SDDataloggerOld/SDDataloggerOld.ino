/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

//SD
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SD.h>
#include <stdint.h>
#include <stdlib.h>
//RTC
#include "Sodaq_DS3231.h"

// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
//LSM9DS1 imu;

const int chipSelect = 8;

long lastVcc = 0;

static long readVcc() { 
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

long filesize = 0;
int logid = 0;
char* logname;

const long long MAX_FILESIZE = 1024ll * 1024ll * 50ll; //50MiB  
const String CSV_HEADER_DATA = "TIME (UTC+0),TIME DELTA,EPOCH,COUNT,VCC";

unsigned long long offset = 0;

static void logln(String tag, String message, boolean newline=true)
{
  if (newline)
    Serial.println(getTimeShort() + " [" + tag + "] " + message);
  else
    Serial.print(getTimeShort() + " [" + tag + "] " + message);
}

/*static void syncClock()
{
  //Goal: compute the average millisecond discrepancy
  logln("TIME", "Syncing high-accuracy clock...");
  
  unsigned long zero = millis();
  unsigned int zero_ts = rtc.now().getEpoch();
  unsigned int next_ts = zero_ts;
  unsigned long long offset = 0;
  for (unsigned int i=0; i<10; i++)
  {
    logln("TIME", "Sync pass " + String(i+1));
    
    do
    {
      next_ts = rtc.now().getEpoch();
    } while (next_ts == zero_ts);
    
    offset += millis() % 1000;
    zero_ts = next_ts;
    
    do
    {
      zero_ts = rtc.now().getEpoch();
    } while (next_ts == zero_ts);
    
    offset += millis() % 1000;
    next_ts = zero_ts;
  }
  offset /= (10 * 2);
  //unsigned long m = millis();
  //unsigned long millisSecond = m - (m % 1000);
  //millisOffset = offset + millisSecond;
  millisOffset = (offset + 15) % 1000;
  logln("TIME", "Offset now " + String(millisOffset) + "ms");
}*/

String getTimeShort()
{
  DateTime now = rtc.now(); //get the current date-time
  unsigned long m = millis();
  String dec = String((m) % 1000, DEC);
  while (dec.length() < 3)
    dec = "0" + dec;
  String secs = String((m - (m % 1000)) / 1000, DEC);
  while (secs.length() < 5)
    secs = "0" + secs;
  String a = (now.hour() < 10 ? String("0") : String("")) + String(now.hour(),DEC) +
             (now.minute() < 10 ? String(":0") : String(":")) + String(now.minute(),DEC) +
             (now.second() < 10 ? String(":0") : String(":")) + String(now.second(),DEC) + 
             " " + secs + "." + dec;
  return a;
}

String getTime()
{
  DateTime now = rtc.now(); //get the current date-time
  uint32_t ts = now.getEpoch(); //get UNIX time
  unsigned long m = millis();
  String dec = String((m) % 1000, DEC);
  while (dec.length() < 3)
    dec = "0" + dec;
  String secs = String((m - (m % 1000)) / 1000, DEC);
  while (secs.length() < 5)
    secs = "0" + secs;
  String a = (now.hour() < 10 ? String("0") : String("")) + String(now.hour(),DEC) +
             (now.minute() < 10 ? String(":0") : String(":")) + String(now.minute(),DEC) +
             (now.second() < 10 ? String(":0") : String(":")) + String(now.second(),DEC) +
             "," + secs + "." + dec + "," + String(ts, DEC);
  return a;
}

static void writeCSVHeader()
{
  //Write CSV header
  File dataFile = SD.open(logname, FILE_WRITE);
  if (dataFile) {
    dataFile.println(CSV_HEADER_DATA);
    dataFile.close();
    logln("LOG", "Successfully wrote CSV header to " + String(logid));
  }
  else {
    logln("SD", "Error opening log with id " + String(logid));
  }
}

static void incrementLogFile()
{
  logid = logid + 1;
  filesize = 0;
  
  //Set logging string
  char lname[16];
  strcpy(lname, "data");
  char lid[8];
  sprintf(lid, "%d", logid);
  strcat(lname, lid);
  strcat(lname, ".csv");
  strcat(lname, "\0");
  
  //free(logname);
  //logname = (char*)malloc(16);
  
  //s.toCharArray(l, s.length());
  strcpy(logname, lname);
  logln("LOG", "Incremented log to " + String(logid) + " ", false);
  Serial.println(logname);
}

static int setupSD()
{
  if (!SD.begin(chipSelect)) {
    logln("SD", "Card failed, or not present");
    return 1;
  }
  
  logln("SD", "Card initialized");
  
  while (SD.exists(logname))
  {
    logln("LOG", "Log already exists!");
    incrementLogFile();
  }

  writeCSVHeader();

  logln("LOG", "Log ready!");

  return 0;
}

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 4 + 2/60 // Declination (degrees) in Austin, Texas

/*static void setupIMU()
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.accel.scale = 16; // Set accel range to +/-16g
  imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  imu.settings.mag.scale = 8; // Set mag range to +/-8Gs
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  logln("IMU", "Starting begin()...");
  while (!imu.begin())
  {
    logln("IMU", "Failed to init IMU.");
    delay(1000);
  }
  logln("IMU", "Init successful!");
}*/

unsigned long count = 0;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  
  // Init RTC
  Wire.begin();
  rtc.begin();
  logln("TIME", "Time is now " + getTime());

  // Init Logging
  logname = (char*)malloc(16);
  strcpy(logname, "data.csv");

  logln("SD", "Initializing SD card...");
  setupSD();

  //logln("IMU", "Initializing IMU...");
  //setupIMU();
  
  //Reset loop count
  count = 1;
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  dataString += getTime();
  dataString += "," + String(count,DEC);
  if (count % 500 == 0)
  {
    lastVcc = readVcc();
  }
  dataString += "," + String(lastVcc);

  // rotate logs if filesize exceeds max size
  filesize = filesize + (long long)(dataString.length() + 2);
  if (filesize >= MAX_FILESIZE)
  {
    incrementLogFile();
    writeCSVHeader();
  }

  //Open file - only one allowed at a time
  File dataFile = SD.open(logname, FILE_WRITE);
  
  //If available, write
  if (dataFile) {
    dataFile.println(dataString);
    logln("DATA", dataString);
    dataFile.close();
    if (!SD.exists(logname))
    {
      logln("SD", "Possible problem writing to card");
    }
  }
  //If not available, print error
  else {
    logln("SD", "Error opening log with id " + String(logid));
    setupSD();
    //incrementLogFile();
  }

  count++;
}









