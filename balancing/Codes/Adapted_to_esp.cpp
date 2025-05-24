#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// MPU6050 object
Adafruit_MPU6050 mpu;

// Define motor control pins
#define LEFT_STEP  16
#define LEFT_DIR   17
#define RIGHT_STEP 4
#define RIGHT_DIR  14

// Stepper motor control objects
step leftMotor(1000, LEFT_STEP, LEFT_DIR);   // interval = 1000us
step rightMotor(1000, RIGHT_STEP, RIGHT_DIR);

float pid_p_gain = 15;
float pid_i_gain = 1.5;
float pid_d_gain = 30;

float pid_error = 0, pid_i_mem = 0, pid_last_d_error = 0, pid_output = 0;
float angle_offset = 0.0;  // Angle offset for calibration

unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL_US = 4000;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Stabilize MPU
  delay(1000);

  // Simple calibration for pitch offset
  float angle_sum = 0;
  for (int i = 0; i < 100; i++) {
    angle_sum += getPitch();
    delay(10);
  }
  angle_offset = angle_sum / 100.0;

  // Set motor acceleration (microsteps/s^2)
  leftMotor.setAcceleration(500);
  rightMotor.setAcceleration(500);
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoopTime >= LOOP_INTERVAL_US) {
    lastLoopTime = now;

    float angle = getPitch() - angle_offset;
    pidController(angle);

    // Set motor speeds based on PID output
    leftMotor.setTargetSpeed((int)pid_output);
    rightMotor.setTargetSpeed((int)pid_output);

    // Update motors
    leftMotor.runStepper();
    rightMotor.runStepper();
  }
}

// Returns pitch angle from MPU6050
float getPitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  return angle;
}

// Simple PID controller
void pidController(float current_angle) {
  float error = current_angle;
  pid_i_mem += pid_i_gain * error;
  pid_i_mem = constrain(pid_i_mem, -400, 400);

  float d_error = error - pid_last_d_error;
  pid_output = pid_p_gain * error + pid_i_mem + pid_d_gain * d_error;
  pid_output = constrain(pid_output, -1000, 1000); // Limit for motor speed

  pid_last_d_error = error;
}