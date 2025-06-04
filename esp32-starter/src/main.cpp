#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include "BluetoothSerial.h"
#include <string>
BluetoothSerial SerialBT;


//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");  // 蓝牙名称
  Serial.println("Bluetooth started");

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
}

std::string command;

void loop() {

      // Fetch data from MPU6050
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if(a.acceleration.z >= 5){
    SerialBT.println("forward");
  }
  else if(a.acceleration.z <= -5){
    SerialBT.println("backward");
  }
  else if(g.gyro.z >= 1){
    SerialBT.println("Right");
  }
  else if(g.gyro.z <= -1){
    SerialBT.println("Left");
  }
  delay(100); 
}
//browse 78:42:1C:6A:D3:62