
#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <step.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;
const int PRINT_INTERVAL    = 300;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;


//Global objects
ESP32Timer ITimer(3);

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


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


void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

 
  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  step1.setAccelerationRad(10);
  step2.setAccelerationRad(10);

  Serial.println("Code started:");


}

void loop()
{

/*
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if(command == "forward") {
      step1.setTargetSpeedRad(100);
      step2.setTargetSpeedRad(100);
    }
    else if (command == "backward") {
      step1.setTargetSpeedRad(-100);
      step2.setTargetSpeedRad(-100);
    }
    else if (command == "left") {
      step1.setTargetSpeedRad(-100);
      step2.setTargetSpeedRad(100);
    }
    else if (command == "right") {
      step1.setTargetSpeedRad(100);
      step2.setTargetSpeedRad(-100);
    }

    else if(command == "forwardANDleft") {
      step1.setTargetSpeedRad(50);
      step2.setTargetSpeedRad(100);
    }
    else if (command == "forwardANDright") {
      step1.setTargetSpeedRad(100);
      step2.setTargetSpeedRad(50);
    }
    else if (command == "backwardANDleft") {
      step1.setTargetSpeedRad(-50);
      step2.setTargetSpeedRad(-100);
    }
    else if (command == "backwardANDright") {
      step1.setTargetSpeedRad(-100);
      step2.setTargetSpeedRad(-50);
    }
    else if (command == "stop") {
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
    }
  
  }

*/

// test motion before serial

  step1.setTargetSpeedRad(1000);
  step2.setTargetSpeedRad(1000);

  delay(1000);

  step1.setTargetSpeedRad(0);
  step2.setTargetSpeedRad(0);

  delay(1000);

  step1.setTargetSpeedRad(-1000);
  step2.setTargetSpeedRad(-1000);

  delay(1000);

  step1.setTargetSpeedRad(0);
  step2.setTargetSpeedRad(0);

}