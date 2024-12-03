#include <Arduino.h>
#include <AS5047P.h>
#include <ArduinoJson.h>
#include <pins_arduino.h>

StaticJsonDocument<1024> doc;
JsonArray encoder_values;
JsonArray encoder_status;

// #define BUS_SPEED 100000 // 100kHz
#define BUS_SPEED 1000000 // 1MHz
// #define BUS_SPEED 10000000 // 10MHz
#define NUM_ENCODERS 7
AS5047P encoders[NUM_ENCODERS] = {
  // Arm 1
  AS5047P(22, BUS_SPEED),
  AS5047P(21, BUS_SPEED), 
  AS5047P(05, BUS_SPEED), 
  AS5047P(17, BUS_SPEED), 
  AS5047P(16, BUS_SPEED), 
  AS5047P(13, BUS_SPEED), 
  AS5047P(12, BUS_SPEED),
};


void setup() {
  Serial.begin(921600);
  encoder_values =  doc.createNestedArray("values");
  encoder_status =  doc.createNestedArray("status");

  for (int i = 0; i < NUM_ENCODERS; i++){
    encoder_values.add(0);
    encoder_status.add(false);
    encoders[i].initSPI();
  }
  pinMode(4, INPUT_PULLUP); // Enable switch
}

void loop() {
  doc["timestamp"] = millis();
  for (int i = 0; i < NUM_ENCODERS; i++){
    encoder_status[i] = encoders[i].checkSPICon();
    encoder_values[i] = encoders[i].readAngleRaw();
  }
  doc["enable_switch"] = !digitalRead(4);
  doc["ID"] = "lightning";
  Serial.print(doc.as<String>() + (char)0);
  delay(30);
}