#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

// mag/accell/gyro input filter
Madgwick filter;

// PWM
unsigned long microsPerReading, microsPrevious;

void setup() {
  Serial.begin(38400);
  Serial.println("Starting setup");

  // join I2C bus
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  mag.initialize();
  barometer.initialize();

  microsPerReading = 50;

  // verify connection I2C devices
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

  Serial.println("Setup complete");
}

void loop() {
    // mag readings
    int16_t mx, my, mz;

    int16_t aix, aiy, aiz;
    int16_t gix, giy, giz;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();
    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    
    // read raw heading measurements from device
    mag.getHeading(&mx, &my, &mz);

    // output raw heading from mag
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");

    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if (heading < 0) heading += 2 * M_PI;
    Serial.println(heading * 180/M_PI); Serial.print("\t");

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&aix, &ay, &aiz, &gix, &giy, &giz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // output quaternion filtered data
    Serial.print(filter.getQuaternion0()); Serial.print("\t");
    Serial.print(filter.getQuaternion1()); Serial.print("\t");
    Serial.print(filter.getQuaternion2()); Serial.print("\t");
    Serial.print(filter.getQuaternion3()); Serial.print("\t");

    // output filtered yaw, pitch and roll
    Serial.print(filter.getRoll()); Serial.print("\t");
    Serial.print(filter.getPitch()); Serial.print("\t");
    Serial.print(filter.getYaw()); Serial.print("\t");

    // increment previous time, so we keep proper pace
    Serial.print(microsPerReading); Serial.print("\t");
    microsPrevious = microsPrevious + microsPerReading;

    Serial.println("\n");
  }
}

// @todo REMOVE Madgwick implements this, confirm first with debug output
float convertRawAcceleration(int16_t aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

// @todo REMOVE Madgwick implements this, confirm first with debug output
float convertRawGyro(int16_t gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
