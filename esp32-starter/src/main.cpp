#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <PID_v1.h>

#define RXD2 16
#define TXD2 17

// The Stepper pins
const int STEPPER1_DIR_PIN  = 23;
const int STEPPER1_STEP_PIN = 25;
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

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

double setpoint = 0;
double input, output;

double Kp = 525.0;
double Ki = 17.0;
double Kd = 19.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastPIDUpdate = 0;
const unsigned long PIDInterval = 15;

float motorSpeed = 0;
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

TaskHandle_t pidTaskHandle;

void pidLoop(void *parameter);
void motorControl();


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
  uint8_t tx0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(tx0);                    // Send Command Byte 0
  uint8_t rx0 = SPI.transfer(tx1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((rx0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Message Serial started on GPIO25(TX), GPIO23(RX)");

  Wire.begin();
  pinMode(TOGGLE_PIN,OUTPUT);
  Serial.println("ESP32 ready to receive messages...");

  // Try to initialize Accelerometer/Gyroscope
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("MPU6050 Found!");

  // mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
  step1.setAccelerationRad(20.0);
  step2.setAccelerationRad(20.0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);
 
  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-10000,10000);
  xTaskCreatePinnedToCore(pidLoop, "PID Task", 10000, NULL, 2, &pidTaskHandle, 0);
  delay(4000);

}


void loop() {
  motorControl();
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    Serial.print("Received Parameters: ");
    Serial.println(incoming);
    sscanf(incoming.c_str(), "%lf %lf %lf %lf", &Kp, &Ki, &Kd, &setpoint);
    Serial.println("Values Set to: ");
    Serial.printf("Kp = %.2f, Ki = %.2f, Kd = %.2f, Setpoint = %.2f\n", Kp, Ki, Kd, setpoint);
  }
}

void pidLoop(void *parameter){
  while(true){
      unsigned long currentMillis = millis();
      if(currentMillis - lastPIDUpdate >= PIDInterval){
        
        lastPIDUpdate = currentMillis;

        // sensors_event_t a, g, temp;
        // mpu.getEvent(&a, &g, &temp);
        // double angle = a.acceleration.z/9.67;

        double angle = 1.0;

        static double smoothedAngle = 0.0;
        smoothedAngle += 0.3 * (angle-smoothedAngle);

        input = smoothedAngle;
        myPID.Compute();

        step1.setTargetSpeedRad(motorSpeed);
        step2.setTargetSpeedRad(-motorSpeed);

        // Serial.print("Angle: ");
        // Serial.print(input);
        // Serial.print("  Motor Speed: ");
        // Serial.print(motorSpeed);
        // Serial.print(step1.getSpeedRad());
        // Serial.println(' ');
    }
    delay(1);
  }
}

void motorControl() {
  // step1.setTargetSpeedRad(motorSpeed);
  // step2.setTargetSpeedRad(-motorSpeed);
}