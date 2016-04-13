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
#include <SPI.h>
#include <SD.h>
#include <stdint.h>
#include <stdlib.h>
//RTC
#include <Wire.h>
#include "Sodaq_DS3231.h"

const int chipSelect = 8;

long readVcc() {
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
const String CSV_HEADER_DATA = "TIME (UTC+0),EPOCH,VCC";
const String CSV_HEADER_LOG = "TIME (UTC+0),TAG,MESSAGE";

String getTimeShort()
{
  DateTime now = rtc.now(); //get the current date-time
  String a = (now.hour() < 10 ? String("0") : String("")) + String(now.hour(),DEC) +
             (now.minute() < 10 ? String(":0") : String(":")) + String(now.minute(),DEC) +
             (now.second() < 10 ? String(":0") : String(":")) + String(now.second(),DEC);
  return a;
}

String getTime()
{
  DateTime now = rtc.now(); //get the current date-time
  uint32_t ts = now.getEpoch(); //get UNIX time
  String a = (now.hour() < 10 ? String("0") : String("")) + String(now.hour(),DEC) +
             (now.minute() < 10 ? String(":0") : String(":")) + String(now.minute(),DEC) +
             (now.second() < 10 ? String(":0") : String(":")) + String(now.second(),DEC) +
             "," + String(ts, DEC);
  return a;
}

void initLogFile()
{
  if (!SD.exists("log.csv"))
  {
    //Write CSV header
    File dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println(CSV_HEADER_LOG);
      dataFile.close();
      Serial.println("[LOG] Successfully wrote CSV header to status log");
    }
    else {
      Serial.println("[SD] Error opening status log");
    }
  }
}

void log(String tag, String message, boolean id=true)
{
  if (id)
    Serial.print(getTimeShort() + " [" + tag + "] " + message);
  else
    Serial.print(message);
  
  File dataFile = SD.open("log.csv", FILE_WRITE);
  if (dataFile) {
    if (id)
      dataFile.print(getTimeShort() + "," + tag + "," + message);
    else
      dataFile.print(message);
    dataFile.close();
  }
  else {
    Serial.println("[LOG] Failed to write to status log");
  }
}

void logln(String tag, String status, boolean id=true)
{
  status += "\r\n";
  log(tag, status, id);
}

void writeCSVHeader()
{
  //Write CSV header
  File dataFile = SD.open(logname, FILE_WRITE);
  if (dataFile) {
    dataFile.println(CSV_HEADER_DATA);
    dataFile.close();
    Serial.println("[LOG] Successfully wrote CSV header to " + String(logid));
  }
  else {
    Serial.println("[SD] Error opening log with id " + String(logid));
  }
}

void incrementLogFile()
{
  logid = logid + 1;
  logln("LOG", "Incrementing log to " + String(logid));
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
  logln("LOG", "Incremented log to " + String(logid));
}

int setupSD()
{
  if (!SD.begin(chipSelect)) {
    Serial.println("[SD] Card failed, or not present");
    return 1;
  }

  initLogFile();
  logln("SD", "Card initialized");
  
  while (SD.exists(logname))
  {
    log("LOG", "Log ");
    log("LOG", logname, false);
    logln("LOG", " already exists!", false);
    incrementLogFile();
  }

  writeCSVHeader();

  log("LOG", "Log ");
  log("LOG", logname, false);
  logln("LOG", " ready!", false);

  return 0;
}

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // Init RTC
  Wire.begin();
  rtc.begin();
  Serial.println("[TIME] Time is now " + getTime());

  // Init Logging
  logname = (char*)malloc(16);
  strcpy(logname, "data.csv");

  Serial.println("[SD] Initializing SD card...");
  setupSD();
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  dataString += getTime();
  dataString += "," + String(readVcc());
  //dataString += "," + String(analogRead(0));

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
    dataFile.close();
    Serial.println(dataString);
    delay(200);
  }
  //If not available, print error
  else {
    logln("SD", "Error opening log with id " + String(logid));
    setupSD();
  }
}









