#include <Arduino.h>

#define LED_PIN 4

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Code started:");
}

void loop() {
    // Read Command
    if(Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if(command == "LED_ON") {
            digitalWrite(LED_PIN, HIGH);
            //Serial.println("LED ON");
        } 
        else if (command == "LED_OFF") {
             digitalWrite(LED_PIN, LOW);
             //Serial.println("LED OFF");
        }
    }
}