#include <Wire.h>
#include <MFRC522_I2C.h>

// The Giga R1 has multiple I2C ports. 
// Wire maps to the SDA/SCL pins (20/21).
// 0x28 is the default I2C address for the M5 RFID Unit.
MFRC522_I2C mfrc522(0x28, -1); 

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open on Giga

  // Initialize I2C (Pins 20/SDA and 21/SCL)
  Wire.begin();

  // Initialize the RFID reader
  mfrc522.PCD_Init();
  
  Serial.println("Giga R1 Ready. Scanning for RFID tags...");
}

int counter = 0;

void loop() {
  // Check for new cards
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    
    // Get the type of card
    uint8_t piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    counter += 1;
    Serial.println(counter);
    Serial.print(F("PICC type: "));
    Serial.println(mfrc522.PICC_GetTypeName(piccType));

    // Check compatibility
    if (piccType != MFRC522_I2C::PICC_TYPE_MIFARE_MINI
        && piccType != MFRC522_I2C::PICC_TYPE_MIFARE_1K
        && piccType != MFRC522_I2C::PICC_TYPE_MIFARE_4K) {
      Serial.println(F("This tag / card is not of type MIFARE Classic.\n"));
      delay(500);
      return;
    }

    // Print UID to Serial Monitor
    Serial.print("UID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      if(mfrc522.uid.uidByte[i] < 0x10) Serial.print("0"); // Leading zero
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println("\n");

    // Halt PICC to stop reading the same card repeatedly
    mfrc522.PICC_HaltA();
    delay(500);
  }
}