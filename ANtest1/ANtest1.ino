#include <MiniMessenger.h>
#include "secrets.h"
#include <Wire.h>
#include <Motoron.h>
MotoronI2C mc;
MiniMessenger messenger;
const char* BoardId = "Kayubo";
const int LED_PIN = 4;
const int BUTTON_PIN = 2;
const int GREEN_LED_PIN = 3;
const int REVIVAL_BUTTON_PIN = 46;
// Two independent enable sources
bool physical_enable = true;   // toggled by button
bool wifi_enable = false;      // set by dashboard heartbeat
// Derived: robot only runs when both are true
bool robotEnabled() {
  return physical_enable && wifi_enable;
}
// MiniMessenger Software Callback
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  // FIX: If the physical button has disabled the robot, ignore ALL server communication
  if (!physical_enable) {
    return;
  }
  if (length == 0) return;
  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';
  bool readable = false;
  for (size_t i = 0; i < length; i++) {
    char c = msg[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      readable = true;
      break;
    }
  }
  if (!readable) return;
  Serial.print("RAW: [");
  Serial.print(msg);
  Serial.println("]");
  // WiFi heartbeat sets wifi_enable only — does NOT touch physical_enable
  if (strstr(msg, "type=heartbeat")) {
    if (strstr(msg, "enable=1")) {
      wifi_enable = true;
      Serial.println("WiFi: ENABLED");
      // Motor enable code here (only acts if physical_enable also true)
    } else if (strstr(msg, "enable=0")) {
      wifi_enable = false;
      Serial.println("WiFi: DISABLED");
      // Motor stop code here
    }
  }
  // Explicit disable from operator
  if (strstr(msg, "type=disable enabled=false reason=operator")) {
    wifi_enable = false;
    Serial.println("WiFi: DISABLED (explicit)");
    // Motor stop code here
  }
}
void move_forward(){
  mc.setSpeed(1,660);
  mc.setSpeed(3,660);
}
void stop(){
  mc.setSpeed(1,0);
  mc.setSpeed(3,0);
}
// Setup
void setup() {
  Serial.begin(115200);
  Wire1.begin();
  mc.setBus(&Wire1); mc.setAddress(0x10);
  mc.reinitialize(); mc.clearResetFlag(); mc.disableCommandTimeout();
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  pinMode(REVIVAL_BUTTON_PIN, INPUT_PULLUP);
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("Messenger Ready!");
}
// Loop
void loop() {
  messenger.loop();
  // Physical button: toggles physical_enable only, single toggle per press
  static unsigned long lastPress = 0;
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW && millis() - lastPress >= 300) {
    lastPress = millis();
    physical_enable = !physical_enable;
    Serial.print("Button: physical_enable = ");
    Serial.println(physical_enable ? "true" : "false");
    // Motor enable/disable code here based on robotEnabled()
  }
  lastButtonState = currentButtonState;
  // LED: solid when enabled, blink when disabled
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  if (robotEnabled()) {
    digitalWrite(LED_PIN, HIGH);
    move_forward();
  } else {
    stop();
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
  }
  // Registration heartbeat: only when physical_enable is true
  if (physical_enable) {
    static unsigned long lastRegister = 0;
    if (millis() - lastRegister >= 10000) {
      lastRegister = millis();
      char reg[64];
      snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
      messenger.sendToBoard("server", reg);
      Serial.println("Registration sent");
    }
  }
  // Revival button: green LED solid only while pressed
  digitalWrite(GREEN_LED_PIN, digitalRead(REVIVAL_BUTTON_PIN) == LOW ? HIGH : LOW);
}