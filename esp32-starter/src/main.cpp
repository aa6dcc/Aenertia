#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 5.0;
const float VREF = 4.096;
// PID 控制参数（你可以根据实际表现调节）
float kp1 = 8848;
float ki1 = 0;
float kd1 = 80 ;//800
float kp2 = -0.003;
float ki2 = -0.00007;
// float kp2 = 0;
// float ki2 = 0;
float kd2 = 0.0;
float kp_turn = 0.0;  
float ki_turn = 0.0;
float kd_turn = 0.0;
float turnIntegral = 0, lastTurnError = 0;

float targetYaw = 0;  // 希望的角度，或者从遥控器获得的转动指令

// PID 状态变量
static float angleIntegral = 0.0;
static float lastAngleError = 0.0;
static float lastAcce=0.0;
static float speedIntegral = 0.0;
static float lastSpeedError = 0.0;
float acce=0.0;
float targetSpeed=0;
float motorCommand=0.0;
float speed = 0.0;
float targetAngle = 0.0;
float output = 0.0;
float  accAngle=0.0;
float  gyroRate=0.0;
float dt=0.0;
float targetAnglebias = 0;
float lastTargetAngle=0.0;
float currentYaw=0.0;
float turnOutput = 0;
//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t TX0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t txByte1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 
   SPI.transfer(TX0);                    // Send Command Byte 0
  uint8_t RX0 = SPI.transfer(TX1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rxByte1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((RX0 & 0x0F) << 8) | rxByte1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
 
  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    if(now<5000){
      targetSpeed=0;
      targetYaw=0;
    }else if (now<9000){
      targetSpeed=8;
      //targetYaw=0.1;
    }else if(now<13000){
      targetSpeed=-8;
      //targetYaw=-0.1;
    }else{
      targetSpeed=0;
      //targetYaw=0;
    }
    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    // Fetch data from MPU6050
    //targetAngle = 0.0;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //tiltx = a.acceleration.z / 9.67;
    //accAngle = atan2(a.acceleration.z, a.acceleration.x);
    accAngle = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));
    gyroRate = g.gyro.y-0.01;
    float alpha = 0.98;
    tiltx =( alpha * (tiltx + gyroRate * dt) + (1 - alpha) * accAngle);
     // === 内环 PID（速度 -> 电机命令）===
    float actualSpeed = -(step1.getSpeedRad());  // 可替换为更合适的速度估计
    float speedError = targetSpeed - actualSpeed;
    speedIntegral += speedError * dt;
    speedIntegral = constrain(speedIntegral, -100, 100);
    float speedDerivative = (speedError - lastSpeedError) / dt;
    targetAngle= (kp2 * speedError + ki2 * speedIntegral + kd2 * speedDerivative)+targetAnglebias;
    targetAngle=0.7*targetAngle+0.3*lastTargetAngle;
    lastTargetAngle=targetAngle;
    lastSpeedError = speedError;
    // === 外环 PID（角度 -> 目标速度）===
    float angleError = targetAngle - tiltx;
    angleIntegral += angleError * dt;
    angleIntegral = constrain(angleIntegral, -300, 300);  // 防止积分饱和
    float angleDerivative = (angleError - lastAngleError) / dt;
    acce = (kp1 * angleError + ki1 * angleIntegral + kd1 * angleDerivative);
    acce= acce*0.7+lastAcce*0.3;
    lastAcce=acce;
    lastAngleError = angleError;

   // ==== 转向 PID 参数 ====

// ==== 获取当前 yaw （偏航角速度或角度） ====
currentYaw = g.gyro.z+0.06;  // 或者集成角度值

// ==== 计算 PID ====
float yawError = targetYaw - currentYaw;
turnIntegral += yawError * dt;
float turnDerivative = (yawError - lastTurnError) / dt;
turnOutput = kp_turn * yawError + ki_turn * turnIntegral + kd_turn * turnDerivative;
lastTurnError = yawError;
if(turnOutput < 0.005 && turnOutput > -0.005){
    turnOutput = 0;
}
else{
    turnOutput = turnOutput;
}


//float error = targetAngle - tiltx;

// 积分项累加
//integral += error * (LOOP_INTERVAL / 1000.0);  // 注意单位：秒

// 微分项
//float derivative = (error - lastError) / (LOOP_INTERVAL / 1000.0);

// PID 控制器输出
//output = (kp1 * error + ki1 * integral + kd1 * derivative)/1000;
//speed = (kp2 * error + ki2 * integral + kd2 * derivative)/1000;
//float maxOutput = 10.0;
//output = constrain(output, -maxOutput, maxOutput);

// 保存上一次误差
//lastError = error;

    //Calculate Tilt using accelerometer and sin x = x approximation for a small tilt angle
    //888tiltx = a.acceleration.z/9.67;

    //Set target motor speed proportional to tilt angle
    //Note: this is for demonstrating accelerometer and motors - it won't work as a balance controller
    acce = constrain(acce, -80, 80);
//   step1.setAccelerationRad(abs(acce+turnOutput));
//   step2.setAccelerationRad(abs(acce-turnOutput));
  step1.setAccelerationRad(abs(acce));
  step2.setAccelerationRad(abs(acce));
  if(acce+turnOutput>0){
     step1.setTargetSpeedRad(-(20));
  }else {
      step1.setTargetSpeedRad((20));
  }
  if(acce-turnOutput>0){
     step2.setTargetSpeedRad((20));
  }else {
      step2.setTargetSpeedRad(-(20));
  }
      
      
//step1.setAccelerationRad(500.0);
//step2.setAccelerationRad(500.0);
//step1.setTargetSpeedRad(-acce);
//step2.setTargetSpeedRad(acce);
//step1.setTargetSpeedRad(-motorCommand);
//step2.setTargetSpeedRad(motorCommand);
}
  
  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer) {
    //if (millis() > printTimer) {
  printTimer += PRINT_INTERVAL;
  // Serial.print("ACC Angle: ");
  // Serial.print(accAngle);
  // Serial.print(" deg\t");
  Serial.print("GYRO Rate: ");
  Serial.print(gyroRate);
  // Serial.print(" dt: ");
  // Serial.print(dt, 4);
  // Serial.println(" deA/s");
  Serial.print(" | tiltx: ");
  Serial.print(tiltx);  // 原本单位不清，现在你要的话可以换成角度显示
  // Serial.print(" | targetAngle: ");
  // Serial.print(targetAngle-targetAnglebias);
  // Serial.print(" | error: ");
  // Serial.print(targetAngle - tiltx);
  // Serial.print(" | output: ");
  // Serial.print(acce);
  // Serial.print(" | turnOutput: ");
  // Serial.print(turnOutput);
//   Serial.print(" | motorSpeed1: ");
//   Serial.print(step1.getSpeedRad());
//    Serial.print(" | motorSpeed2: ");
//   Serial.print(step2.getSpeedRad());
  Serial.print(" | targetSpeed: ");
  Serial.print(targetSpeed);
  Serial.print(" |gyro.z: ");
  Serial.print(currentYaw);
  // Serial.print(" | position1: ");
  // Serial.print(step1.getPosition());
  // Serial.print(" | position2: ");
  // Serial.print(step2.getPosition());
  //Serial.print(" | millis: ");
  //Serial.print(millis());
  Serial.print(" | ADC(A0): ");
  Serial.println(((readADC(0) * VREF) / 4095.0-0.21)/1.5);
//   Serial.print(" | TargetYaw: ");
//   Serial.println(targetYaw);
}

  //}
}