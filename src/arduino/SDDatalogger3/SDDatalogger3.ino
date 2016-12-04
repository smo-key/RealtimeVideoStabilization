//SD
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#define chipSelect 10

//IMU
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU sixDOF = FreeSixIMU();

//Voltage regulator
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
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//long filesize = 0;
int logid = 0;
char* logname;

const long MAX_FILESIZE = 1024l;// * 1024l; //1MiB
#define CSV_HEADER_DATA "UPTIME,COUNT,VCC,YAW,PITCH,ROLL"
#define PRINT_DEBUG

static void logln(String tag, String message)
{
#ifdef PRINT_DEBUG
  Serial.println(getTime() + " [" + tag + "] " + message);
#endif
}

static String getTime()
{
  unsigned long m = millis();
  String dec = String((m) % 1000, DEC);
  while (dec.length() < 3)
    dec = "0" + dec;
  String secs = String((m - (m % 1000)) / 1000, DEC);
  while (secs.length() < 5)
    secs = "0" + secs;
  String a = secs + "." + dec;
  return a;
}

static void writeCSVHeader()
{
  //Write CSV header
  File dataFile = SD.open(logname, FILE_WRITE);
  if (dataFile) {
    dataFile.println(CSV_HEADER_DATA);
    dataFile.close();
    logln("LOG", "Wrote CSV header to " + String(logid));
  }
  else {
    logln("SD", "Error opening log " + String(logid));
  }
}

static void incrementLogFile()
{
  logid = logid + 1;

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
    logln("SD", "Card failed!");
    return 1;
  }

  logln("SD", "Card initialized!");

  while (SD.exists(logname))
  {
    logln("LOG", "Log already exists!");
    incrementLogFile();
  }

  writeCSVHeader();

  logln("LOG", "Log ready!");

  return 0;
}

void initIMU()
{
  delay(5000);

  delay(5);
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
}

String readIMU()
{
  float angles[3]; // yaw pitch roll

  sixDOF.getYawPitchRoll(angles);

  return String(angles[0], DEC) + "," + String(angles[1], DEC)
         + "," + String(angles[2], DEC);
}

unsigned long count = 0;

boolean okay = false;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin();

  // Init Logging
  logname = (char*)malloc(16);
  strcpy(logname, "data.csv");

  logln("SD", "Initializing SD card...");
  setupSD();

  logln("IMU", "Initializing IMU...");
  initIMU();

  //Reset loop count
  count = 1;

  //Setup data pins
  pinMode(4, OUTPUT); //Status LED

  Serial.println("Init complete.");
}

void loop()
{
  digitalWrite(4, okay && (count % 25 > 1));
  if (count % 100 == 1)
  {
    lastVcc = readVcc();
  }
  //Open file - only one allowed at a time
  File dataFile = SD.open(logname, FILE_WRITE);

  //If available, write
  if (dataFile) {
    dataFile.println(getTime() + ","
                     + String(count, DEC) + ","
                     + String(lastVcc) + ","
                     + readIMU());
    dataFile.close();
    logln("DATA", "Received");
    okay = true;
  }
  else
  {
    okay = false;
    logln("SD", "Error opening log " + String(logid));
    setupSD();
  }

  count++;
}




