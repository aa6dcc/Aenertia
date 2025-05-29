
#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <step.h>

#define LED_PIN 2


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
float spd = 8.0;
String mode = "manual";

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
  Serial1.begin(115200, SERIAL_8N1, 9, 10); // RX = GPIO9, TX = GPIO10
  Serial1.println("Hello from debug port");

  pinMode(TOGGLE_PIN,OUTPUT);

 
  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    //Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  //Serial.println("Initialised Interrupt for Stepper");

  step1.setAccelerationRad(20);
  step2.setAccelerationRad(20);


  //LED debug output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //Serial.println("Code started:");


}

void loop()
{
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
  
    // We start by checking if the mode was changed
    Serial1.println("THIS IS ONLY FOR THE ESP")
    if(command == "forward") {
      step1.setTargetSpeedRad(spd);
      step2.setTargetSpeedRad(spd);
      Serial.println("forward");
    }
    else if (command == "backward") {
      step1.setTargetSpeedRad(-spd);
      step2.setTargetSpeedRad(-spd);
      Serial.println("backward");
    }
    else if (command == "left") {
      step1.setTargetSpeedRad(-spd);
      step2.setTargetSpeedRad(spd);
      Serial.println("left");
    }
    else if (command == "right") {
      step1.setTargetSpeedRad(spd);
      step2.setTargetSpeedRad(-spd);
      Serial.println("right");
    }

    else if(command == "forwardANDleft") {
      step1.setTargetSpeedRad(4*spd/11);
      step2.setTargetSpeedRad(spd);

      Serial.println("Forward and left");

    }
    else if (command == "forwardANDright") {
      step1.setTargetSpeedRad(spd);
      step2.setTargetSpeedRad(4*spd/11);

      Serial.println("Forward and right");
    }
    else if (command == "backwardANDleft") {
      step1.setTargetSpeedRad(-4*spd/11);
      step2.setTargetSpeedRad(-spd);
      Serial.println("Backward and left");
    }
    else if (command == "backwardANDright") {
      step1.setTargetSpeedRad(-spd);
      step2.setTargetSpeedRad(-4*spd/11);
      Serial.println("Backward and right");
    }
    else if (command == "stop") {
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
      Serial.println("Stop");
    }
  
  }

/* test motion before serial
  digitalWrite(LED_PIN, HIGH);
  step1.setTargetSpeedRad(10);
  step2.setTargetSpeedRad(10);
  Serial.println("Forward");

  delay(4000);

  digitalWrite(LED_PIN, LOW);
  step1.setTargetSpeedRad(0);
  step2.setTargetSpeedRad(0);
  Serial.println("Stop");

  delay(4000);

  digitalWrite(LED_PIN, HIGH);
  step1.setTargetSpeedRad(-10);
  step2.setTargetSpeedRad(10);
  Serial.println("Backward");

  delay(4000);

  digitalWrite(LED_PIN, LOW);
  step1.setTargetSpeedRad(0);
  step2.setTargetSpeedRad(0);
  Serial.println("Stop");
  delay(4000);
*/

}