#include <MiniMessenger.h>
#include "secrets.h"

MiniMessenger messenger;

const char* BoardId = "Kayubo";

// =====================================================
// Runs whenever a message is received
// =====================================================

void onMessage(const MessageMetadata& metadata,
               const uint8_t* payload,
               size_t length)
{
  // Ignore empty packets
  if (length == 0)
  {
    return;
  }

  // Convert payload into string
  char msg[256];

  if (length >= sizeof(msg))
  {
    length = sizeof(msg) - 1;
  }

  memcpy(msg, payload, length);
  msg[length] = '\0';

  // =====================================================
  // Filter out garbage/binary packets
  // Only allow messages with readable letters/numbers
  // =====================================================

  bool readable = false;

  for (size_t i = 0; i < length; i++)
  {
    char c = msg[i];

    if (
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9')
    )
    {
      readable = true;
      break;
    }
  }

  // Ignore junk packets
  if (!readable)
  {
    return;
  }

  // =====================================================
  // Print clean message
  // =====================================================

  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(msg);

  // =====================================================
  // Detect disable command
  // =====================================================

  if (strstr(msg, "type=disable enabled=false reason=operator"))
  {
    Serial.println("Robot Disabled!");

    // Put disable code here
    // Example:
    // digitalWrite(MOTOR_ENABLE_PIN, LOW);
  }
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  Serial.begin(115200);

  // Register callback
  messenger.onMessage(onMessage);

  // Start messenger
  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    BoardId
  );

  Serial.println("Messenger Ready!");
}

// =====================================================
// Main loop
// =====================================================

void loop()
{
  // Keep MQTT/messenger alive
  messenger.loop();

  // Send registration every 10 seconds
  static unsigned long lastRegister = 0;

  if (millis() - lastRegister >= 10000)
  {
    lastRegister = millis();

    char reg[64];

    snprintf(
      reg,
      sizeof(reg),
      "type=register team_id=%s board_id=%s",
      GROUP_ID,
      BoardId
    );

    messenger.sendToBoard("server", reg);

    Serial.println("Registration sent");
  }
}
