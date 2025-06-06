#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>


//ADC pins
const int ADC_CS_PIN        = 5;  
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

const int change_interval = 500;
int lastTime = 0;
int variable = 0;
int ADC_value = 0;
int readADC_count = 0;
String output;
String finalOutput = "";

float sim[16] = {1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6};
int loopCount = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// put function declarations here:
int myFunction(int, int);

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


void setup() {
  // put your setup code here, to run once:


// Set the LCD address to 0x27 (sometimes 0x3F)
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() > lastTime + change_interval){
    lastTime = millis();
    float voltage = sim[loopCount];
    float v5 = sim[loopCount + 5];
    float current_motor = sim[loopCount+3];
    float current_board = sim[loopCount+6];

    // Create a JSON object
    StaticJsonDocument<128> doc;
    doc["VB"] = voltage;
    doc["V5"] = v5;
    doc["CM"] = current_motor;
    doc["CB"] = current_board;

    // Serialize JSON to a string
    serializeJson(doc, output);
    // Send JSON over custom serial

    ADC_value = (int)readADC(1);
    lcd.clear();
    lcd.print("VM: ");
    lcd.print(voltage);
    variable += 1;
    // Serial.println("Message Sent");
    Serial.print("PM: ");
    Serial.println(output);

    if(loopCount == 9){
      loopCount = 0;
    }else{
      loopCount += 1;
    }
  }
}