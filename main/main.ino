#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

uint32_t previousLoopUs = 0;
uint32_t batchStartUs = 0;
uint32_t loopCount = 0;
const uint32_t LOOP_PERIOD_US = 4000;

float Ax, Ay, Az;

float Gx, Gy, Gz;

float xMax = 1.0;
float xMin = -0.99;
float yMax = 1.02;
float yMin = -1.00;
float zMax = 0.96;
float zMin = -1.07;

float xOffset = (xMax + xMin) / 2;
float yOffset = (yMax + yMin) / 2;
float zOffset = (zMax + zMin) / 2;

float xScale = 2 / (xMax - xMin);
float yScale = 2 / (yMax - yMin);
float zScale = 2 / (zMax - zMin);

float rollLP;
float pitchLP;

float rollRAW;
float pitchRAW;

float rollG = 0;
float pitchG = 0;
float yawG = 0;

float rollComp = 0;
float pitchComp = 0;
float yawComp = 0;

float deltaRoll = 0;
float deltaPitch = 0;

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

uint32_t tStart = micros();

Adafruit_MPU6050 mpu;

void calibrateGyro() {
  Serial.println("Calibrating gyro, please keep circuit stationary");
  delay(1000);
  int i;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  int numPoints = 1000;

  for (i = 0; i < numPoints; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX = sumX + g.gyro.x;
    sumY = sumY + g.gyro.y;
    sumZ = sumZ + g.gyro.z;
    delay(10);
  }

  gyroXOffset = sumX / numPoints;
  gyroYOffset = sumY / numPoints;
  gyroZOffset = sumZ / numPoints;
}

void calculateLoopFrequency () {
  uint32_t nowUs = micros();

  if (loopCount == 0) {
    batchStartUs = nowUs;
  }

  loopCount++;

  if (loopCount >= 1000) {
    uint32_t elapsedUs = nowUs - batchStartUs;

    float frequencyHz =
      (loopCount - 1) * 1000000.0f / elapsedUs;

    Serial.print("Average loop frequency: ");

    Serial.print(frequencyHz);

    Serial.println(" Hz");

    loopCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (true) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro();
}

void loop() {
  uint32_t currentUs = micros();

  if (currentUs - previousLoopUs < LOOP_PERIOD_US) {
    return;
  }

  previousLoopUs = currentUs;

  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  Ax = a.acceleration.x / 9.81;
  Ay = a.acceleration.y / 9.81;
  Az = a.acceleration.z / 9.81;

  Gx = g.gyro.x - gyroXOffset;
  Gy = -(g.gyro.y - gyroYOffset);
  Gz = g.gyro.z - gyroZOffset;

  Ax = xScale * (Ax - xOffset);
  Ay = yScale * (Ay - yOffset);
  Az = zScale * (Az - zOffset);

  pitchRAW = atan2(Ay, sqrt(Az * Az + Ax * Ax)) * 360 / (2 * 3.14);
  rollRAW = atan2(Ax, sqrt(Az * Az + Ay * Ay)) * 360 / (2 * 3.14);

  pitchLP = .85 * pitchLP + .15 * pitchRAW;
  rollLP = .85 * rollLP + .15 * rollRAW;

  float dt = micros() - tStart;

  // GYRO calculation for roll, pitch, yaw
  rollG = rollG + (dt)*1e-6f * Gy * 360. / 2. / 3.14;
  pitchG = pitchG + (dt)*1e-6f * Gx * 360. / 2. / 3.14;
  yawG = yawG + (dt)*1e-6f * Gz * 360. / 2. / 3.14;

  deltaRoll = (dt)*1e-6f * Gy * 360. / 2. / 3.14;
  deltaPitch = (dt)*1e-6f * Gx * 360. / 2. / 3.14;

  tStart = micros();

  rollComp = .1 * rollRAW + .9 * (rollComp + deltaRoll);
  pitchComp = .1 * pitchRAW + .9 * (pitchComp + deltaPitch);

  // Serial.print("rollComp:");
  // Serial.print(rollComp);
  // Serial.print(",");
  // Serial.print("pitchComp:");
  // Serial.print(pitchComp);
  // Serial.print(",");
  // Serial.print("yawG:");
  // Serial.print(yawG);
  // Serial.print(",");
  // Serial.print("UL:");
  // Serial.print(90);
  // Serial.print(",");
  // Serial.print("LL:");
  // Serial.println(-90);
}