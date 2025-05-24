#include <Arduino.h>

#define LED_PIN 2

void setup() {
    Serial.begin(2500);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}


void loop() {
    // Read Command
    if(Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if(command == "LED_ON") {
            digitalWrite(LED_PIN, HIGH);
        }
        else if (command == "LED_OFF") {
             digitalWrite(LED_PIN, LOW);
        }
    }
}