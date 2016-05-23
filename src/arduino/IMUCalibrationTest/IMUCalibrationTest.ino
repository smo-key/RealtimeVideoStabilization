#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

//Magnetic declination
#define DECLINATION 4 + 2/60 // Declination (degrees) in Austin, Texas

 
float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;
 
float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;
 
long lastDisplayTime;
 
void setup(void) 
{
  Serial.begin(115200);
  
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
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

double ERROR_GYRO = 0.0;
double ERROR_ACCEL = 0.0;
double ERROR_MAG = 0.0;
double OFFSET_GYRO_X = 0.0;
double OFFSET_GYRO_Y = 0.0;
double OFFSET_GYRO_Z = 0.0;
double OFFSET_ACCEL_X = 0.0;
double OFFSET_ACCEL_Y = 0.0;
double OFFSET_ACCEL_Z = 0.0;
double OFFSET_MAG_X = 0.0;
double OFFSET_MAG_Y = 0.0;
double OFFSET_MAG_Z = 0.0;

void cal_antidrift()
{
  
}
 
void loop(void) 
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();
  
  if (imu.ax < AccelMinX) AccelMinX = imu.ax;
  if (imu.ax > AccelMaxX) AccelMaxX = imu.ax;
  
  if (imu.ay < AccelMinY) AccelMinY = imu.ay;
  if (imu.ay > AccelMaxY) AccelMaxY = imu.ay;
 
  if (imu.az < AccelMinZ) AccelMinZ = imu.az;
  if (imu.az > AccelMaxZ) AccelMaxZ = imu.az;
 
  if (imu.mx < MagMinX) MagMinX = imu.mx;
  if (imu.mx > MagMaxX) MagMaxX = imu.mx;
  
  if (imu.my < MagMinY) MagMinY = imu.my;
  if (imu.my > MagMaxY) MagMaxY = imu.my;
 
  if (imu.mz < MagMinZ) MagMinZ = imu.mz;
  if (imu.mz > MagMaxZ) MagMaxZ = imu.mz;

  Serial.print("Magnetometer 
 
  if ((millis() - lastDisplayTime) > 1000)  // display once/second
  {
    Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
    Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
    Serial.print("Mag Minimums: "); Serial.print(MagMinX); Serial.print("  ");Serial.print(MagMinY); Serial.print("  "); Serial.print(MagMinZ); Serial.println();
    Serial.print("Mag Maximums: "); Serial.print(MagMaxX); Serial.print("  ");Serial.print(MagMaxY); Serial.print("  "); Serial.print(MagMaxZ); Serial.println(); Serial.println();
    lastDisplayTime = millis();
  }
}
