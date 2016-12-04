//SD
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#define chipSelect 10

//IMU
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

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

//const long MAX_FILESIZE = 1024l;// * 1024l; //1MiB
#define CSV_HEADER_DATA "UPTIME,COUNT,VCC,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ"
//#define PRINT_DEBUG

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
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.accel.scale = 16; // Set accel range to +/-16g
  imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  imu.settings.mag.scale = 8; // Set mag range to +/-8Gs
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Error initializing IMU.");
  }
}

String readIMU()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  return String(imu.gx) + "," + String(imu.gy) + "," +
         String(imu.gz) + "," + String(imu.ax) + "," +
         String(imu.ay) + "," + String(imu.az) + "," +
         String(imu.mx) + "," + String(imu.my) + "," +
         String(imu.mz);
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




