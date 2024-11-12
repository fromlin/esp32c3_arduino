#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti WiFiMulti;

// WiFi network name and password:
const char *networkName = "SNUH_CMTI";
const char *networkPswd = "@cmtiwifi1@";

// Use NetworkClient class to create TCP connections
const char *host = "10.104.28.71";
const int port = 3333;
NetworkClient client;
boolean connected = false;

#define LED_BUILTIN 8
#define DEBUT_SERIAL Serial
#define DXL_SERIAL Serial0

void setup() {
  // Initialize GPIO pins
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize hardware serial:
  Serial.begin(115200);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
  delay(500);
}

int idx = 0;
void loop() {
  if (connected) {
    if (!client.connected()) {
      client.stop();
      if (!client.connect(host, port)) {
        Serial.println("Connection failed.");
        Serial.println("Waiting 1 seconds before retrying...");
        delay(1000);
      } else {
        Serial.println("TCP connected!");
      }
    } else {
      client.printf("test: %d\n", idx++);
      if (client.available() > 0) {
        String line = client.readStringUntil('\n');
        Serial.print("echo ");
        Serial.println(line);
      }
    }
    delay(100);
  }
}


void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Connecting to ");
      Serial.println(host);
      while (!client.connect(host, port)) {
        Serial.println("Connection failed.");
        Serial.println("Waiting 1 seconds before retrying...");
        delay(1000);
      }
      Serial.println("TCP connected!");
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      Serial.println("Closing connection.");
      client.stop();
      connected = false;
      break;
    default: break;
  }
}