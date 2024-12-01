#include <Arduino.h>
#include <ArduinoJson.h>
#include <FlashStorage_STM32.h>

# define RX_BUFSIZE 2048

const uint32_t SIGNATURE = 0x12345678;
const int DEV_NUM_PINS = 6;
const size_t DEV_NAME_SIZE = 128;
const uint16_t DEFAULT_PWM_PINS[DEV_NUM_PINS] = {A0, A1, A2, A3, A6, A7};
uint16_t pwmPins[DEV_NUM_PINS] = {A0, A1, A2, A3, A6, A7};
char name[DEV_NAME_SIZE]= "Default";

void serial_RX();
void handle_message();
void save_settings();
bool load_settings();

size_t rx_len;
byte rx_buffer[RX_BUFSIZE];
JsonDocument tx_doc;
JsonDocument rx_doc;

const char* error = "";

void setup() {
  Serial.begin(921600);
  Serial.setTimeout(0);

  if (!load_settings()) {
    tx_doc["NOTE"] = "Settings Defaulted";
  }

  rx_len = 0;

  for (int i = 0; i < DEV_NUM_PINS; i++)
    pinMode(pwmPins[i], OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  serial_RX();

  if (strlen(error) > 0){
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void serial_RX(){
  size_t len_read = Serial.readBytes((char*)(rx_buffer+rx_len), RX_BUFSIZE-rx_len);
  rx_len += len_read;

  if (len_read != 0) {
    if (rx_len >= RX_BUFSIZE) {
      rx_len = 0;
    }

    if (rx_len > 0) {
      void* end_term = memchr(rx_buffer, 0, rx_len);
      if (end_term != NULL) {
        // Parse JSON
        DeserializationError json_error = deserializeJson(rx_doc, rx_buffer, (size_t)((byte*)end_term - rx_buffer));
        if (json_error) {
          error = "JSON Parse Error";
          Serial.write(rx_buffer, rx_len);
        }
        // move remaining bytes to beginning of buffer
        size_t remaining = rx_len - (size_t)((byte*)end_term - rx_buffer + 1);
        memmove(rx_buffer, end_term, remaining);
        rx_len = remaining;

        // Handle JSON
        tx_doc.clear();
        handle_message();
        if (strlen(error) > 0)
          tx_doc["ERROR"] = error;
        serializeJson(tx_doc, Serial);
        Serial.write(uint8_t(0));
        error = "";
      }
    }
  }

}

void handle_message(){
  if (rx_doc.containsKey("ID")) {
    tx_doc["ID"] = rx_doc["ID"];
  } else {
    error = "No ID";
    return;
  }

  // Handle PWM message
  if (rx_doc.containsKey("PWM")) {
    if (rx_doc["PWM"].is<JsonArray>()) {
      JsonArray pwm = rx_doc["PWM"].as<JsonArray>();
      for (int i = 0; i < DEV_NUM_PINS; i++) {
        if (pwm.size() > i) {
          if (pwm[i].is<float>()) {
            int pwm_val = pwm[i].as<float>() * 255;
            if (pwm_val >= 0 && pwm_val <= 255) {
              analogWrite(pwmPins[i], pwm_val);
              pwm[i] = pwm_val;
            }
          }
        }
      }
      tx_doc["PWM"] = pwm;
      }
  }

  // Handle info message
  if(rx_doc.containsKey("Info")) {
    tx_doc["NAME"] = (char*)name;
  }

  // Handle save message
  if(rx_doc.containsKey("SAVE")) {
    tx_doc["OLD_NAME"] = (char*)name;
    tx_doc["OLD_PWM"] = JsonArray();
    for (int i = 0; i < DEV_NUM_PINS; i++) {
      tx_doc["OLD_PWM"].add(pwmPins[i]);
    }

    if(rx_doc["SAVE"].is<JsonObject>()) {
      JsonObject save = rx_doc["SAVE"].as<JsonObject>();
      if(save.containsKey("NAME")) {
        if(save["NAME"].is<const char*>()) {
          strncpy(name, save["NAME"], DEV_NAME_SIZE);
        }
      }
      if(save.containsKey("PWM_ORDER")) {
        if(save["PWM_ORDER"].is<JsonArray>()) {
          JsonArray pwm = save["PWM_ORDER"].as<JsonArray>();
          for (int i = 0; i < DEV_NUM_PINS; i++) {
            if (pwm.size() > i) {
              if (pwm[i].is<int>()) {
                pwmPins[i] = DEFAULT_PWM_PINS[pwm[i].as<int>()];
              }
            }
          }
        }
      }
      save_settings();
      tx_doc["SAVE"] = "OK";
      tx_doc["NEW_NAME"] = (char*)name;
      tx_doc["NEW_PWM"] = JsonArray();
      for (int i = 0; i < DEV_NUM_PINS; i++) {
        tx_doc["NEW_PWM"].add(pwmPins[i]);
      }
    }
  }
}

bool load_settings(){
  size_t pos = 0;
  uint32_t signature;
  EEPROM.get(pos, signature);
  pos += sizeof(signature);

  if (SIGNATURE != signature) {
    return false;
  }

  EEPROM.get(pos, name);
  pos += DEV_NAME_SIZE;

  for(int i = 0; i < DEV_NUM_PINS; i++){
    EEPROM.get(pos, pwmPins[i]);
    pos += sizeof(pwmPins[i]);
  }
  return true;
}

void save_settings(){
  size_t pos = 0;

  EEPROM.put(pos, SIGNATURE);
  pos += sizeof(SIGNATURE);

  EEPROM.put(pos, name);
  pos += DEV_NAME_SIZE;

  for(int i = 0; i < DEV_NUM_PINS; i++){
    EEPROM.put(pos, pwmPins[i]);
    pos += sizeof(pwmPins[i]);
  }

  EEPROM.commit();
}