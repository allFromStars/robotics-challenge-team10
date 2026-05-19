#include <Wire.h>
#include <MFRC522_I2C.h>

#define RFID_ADDR 0x28

MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

bool startupMessagePrinted = false;


void setup() {

  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  delay(1000);  

  Wire1.begin(); 
  delay(500);

  rfid.PCD_Init();
  delay(100);

  Serial.println();
  Serial.println("WS1850S RFID reader ready");
  Serial.println("Place card near reader...");
  Serial.println();

  startupMessagePrinted = true;
}


void loop() {

  if (!startupMessagePrinted) {
    Serial.println("WS1850S RFID reader ready");
    Serial.println("Place card near reader...");
    startupMessagePrinted = true;
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    delay(100);
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    delay(100);
    return;
  }

  Serial.print("UID: ");

  for (byte i = 0; i < rfid.uid.size; i++) {

    if (rfid.uid.uidByte[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  delay(1000);
}
