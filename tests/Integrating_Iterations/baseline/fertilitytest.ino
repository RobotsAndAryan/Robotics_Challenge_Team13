#include <Wire.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>

#define WIFI_SSID "PhaseSpaceNetwork_2.4G"
#define WIFI_PASSWORD "8igMacNet"
#define BROKER_HOST "192.168.0.74" // Lab MQTT Broker
#define BROKER_PORT 1883
#define GROUP_ID "13"  // Use your assigned team number

// The Giga R1 has multiple I2C ports. 
// Wire maps to the SDA/SCL pins (20/21).
// 0x28 is the default I2C address for the M5 RFID Unit.
MFRC522_I2C mfrc522(0x28, -1); 
MiniMessenger messenger;

// Your unique board identifier (Ensure there are no spaces!)
const char* BoardId = "Kayubo1";

// Global variable to throttle heartbeat logs
unsigned long lastHeartbeatPrint = 0;

void onMessage(const MessageMetadata& metadata,
               const uint8_t* payload,
               size_t length)
{
  // Ignore empty packets
  if (length == 0) return;

  // Convert payload into string safely
  char msg[256];
  if (length >= sizeof(msg)) {
    length = sizeof(msg) - 1;
  }
  memcpy(msg, payload, length);
  msg[length] = '\0';

  // =====================================================
  // Filter out garbage/binary packets
  // =====================================================
  bool readable = false;
  for (size_t i = 0; i < length; i++) {
    char c = msg[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      readable = true;
      break;
    }
  }

  // Ignore junk packets
  if (!readable) return;

  // =====================================================
  // Handle Heartbeat (Sent by server every 250ms)
  // =====================================================
  if (strstr(msg, "type=heartbeat"))
  {
    // Print heartbeat payload only once every 5 seconds to prevent Serial flooding
    if (millis() - lastHeartbeatPrint >= 5000) {
      lastHeartbeatPrint = millis();
      Serial.print("[5s Heartbeat Check] Incoming data: ");
      Serial.println(msg);
    }

    // Safety critical: Check the enable flag on EVERY packet regardless of the print timer
    if (strstr(msg, "enable=0")) {
      Serial.println("Heartbeat: STOP commanded!");
      // STOP YOUR MOTORS HERE IMMEDIATELY
    }
    return; // Exit callback early for heartbeats
  }

  // =====================================================
  // Print all other clean non-heartbeat messages
  // =====================================================
  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(msg);

  // =====================================================
  // Detect disable command
  // =====================================================
  if (strstr(msg, "type=disable enabled=false reason=operator")) {
    Serial.println("Robot Disabled!");
    // Put disable code here (e.g., digitalWrite(MOTOR_ENABLE_PIN, LOW);)
  }

  // =====================================================
  // Handle isFertileReply
  // =====================================================
  if (strstr(msg, "type=isFertileReply")) {
    Serial.println("Received fertility status from server:");
    Serial.println(msg);
    
    if (strstr(msg, "fertile=true") && strstr(msg, "planted=false")) {
      Serial.println("=> Tag is FERTILE and NOT planted. Ready to plant!");
      // Add your planting execution logic here
    } else if (strstr(msg, "planted=true")) {
      Serial.println("=> Tag already planted!");
    } else if (strstr(msg, "fertile=false")) {
      Serial.println("=> Tag is NOT fertile!");
    }
  }

  // =====================================================
  // Handle emergency stop
  // =====================================================
  if (strstr(msg, "type=emergency enabled=true")) {
    Serial.println("EMERGENCY STOP!");
    // Immediate physical hardware stop here
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open on Giga R1

  // Initialize I2C (Pins 20/SDA and 21/SCL)
  Wire.begin();

  // Initialize the RFID reader
  mfrc522.PCD_Init();
  Serial.println("Giga R1 Ready. Scanning for RFID tags...");

  // Register MQTT message callback
  messenger.onMessage(onMessage);

  // Start messenger layer using variables predefined inside secrets.h
  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    BoardId
  );

  Serial.println("Messenger Initialization complete.");
}

int counter = 0;

void loop() {
  // Keep connection alive and poll for incoming MQTT packets
  messenger.loop();

  // Registration loop (Every 10 seconds - Mandatory to stay "Online")
  static unsigned long lastRegister = 0;
  if (millis() - lastRegister >= 10000) {
    lastRegister = millis();

    // Check if network client is online before pushing data
    if (messenger.isConnected()) {
      char reg[64];
      
      // Using %s assuming GROUP_ID inside secrets.h is a string literal (e.g. "5")
      // If GROUP_ID is an integer (e.g. 5), change %s below to %d
      snprintf(
        reg,
        sizeof(reg),
        "type=register team_id=%s board_id=%s",
        GROUP_ID,
        BoardId
      );

      messenger.sendToBoard("server", reg);
      Serial.print("Registration payload sent: ");
      Serial.println(reg);
    } else {
      Serial.println("Registration delayed: Waiting for connection to broker...");
    }
  }

  // --- Read RFID Card Logic --- // 
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    
    uint8_t piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    counter += 1;
    Serial.print("Card Read #");
    Serial.println(counter);
    Serial.print(F("PICC type: "));
    Serial.println(mfrc522.PICC_GetTypeName(piccType));

    // Compatibility validation
    if (piccType != MFRC522_I2C::PICC_TYPE_MIFARE_MINI
        && piccType != MFRC522_I2C::PICC_TYPE_MIFARE_1K
        && piccType != MFRC522_I2C::PICC_TYPE_MIFARE_4K) {
      Serial.println(F("This tag / card is not of type MIFARE Classic.\n"));
      mfrc522.PICC_HaltA();
      delay(500);
      return;
    }

    // Build uppercase Hex string for UID
    char tag_id[32] = "";
    Serial.print("UID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      if(mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      Serial.print(" ");
      
      char hex[3];
      sprintf(hex, "%02X", mfrc522.uid.uidByte[i]);
      strcat(tag_id, hex);
    }
    Serial.println();

    // Build isFertile string matching challenge formatting protocols
    char queryMsg[64];
    snprintf(
      queryMsg,
      sizeof(queryMsg),
      "type=isFertile tag_id=%s board_id=%s",
      tag_id,
      BoardId
    );
    
    if (messenger.isConnected()) {
      messenger.sendToBoard("server", queryMsg);
      Serial.print("Sent to server: ");
      Serial.println(queryMsg);
    } else {
      Serial.println("Could not query server: MQTT Disconnected.");
    }

    // Halt PICC to stop reading duplicate data frames from the same card
    mfrc522.PICC_HaltA();
    delay(500);
  }
}
