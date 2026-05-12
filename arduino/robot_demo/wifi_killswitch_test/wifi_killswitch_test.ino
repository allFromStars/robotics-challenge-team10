#include <WiFi.h>
#include <WiFiUdp.h>

// =========================
// WIFI SETTINGS
// =========================

const char* ssid = "Xia";
const char* password = "He600910";

// =========================
// UDP
// =========================

WiFiUDP udp;

const int UDP_PORT = 4210;

char packetBuffer[255];

// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("Starting WiFi UDP test...");

  // Connect to WiFi
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");

  // Print Arduino IP address
  Serial.print("Arduino IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP listener
  udp.begin(UDP_PORT);

  Serial.print("Listening for UDP packets on port ");
  Serial.println(UDP_PORT);

  Serial.println();
  Serial.println("Send UDP message now...");
}

// =========================
// LOOP
// =========================

void loop() {

  int packetSize = udp.parsePacket();

  // If packet received
  if (packetSize) {

    int len = udp.read(packetBuffer, 255);

    if (len > 0) {
      packetBuffer[len] = 0;
    }

    Serial.print("Received packet: ");
    Serial.println(packetBuffer);
  }
}