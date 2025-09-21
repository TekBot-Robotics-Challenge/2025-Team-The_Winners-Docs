#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// === Configuration Wi-Fi ===
// const char* ssid = "Fruc S10+";
// const char* password = "obtne20220";

// const char* ssid = "Moov -Africa";         // Remplacez par le nom de votre réseau Wi-Fi
// const char* password = "@@password@"; // Remplacez par le mot de passe de votre réseau Wi-Fi

const char* ssid = "rasptank";         // Remplacez par le nom de votre réseau Wi-Fi
const char* password = "azerty1234"; // Remplacez par le mot de passe de votre réseau Wi-Fi

// === Configuration du serveur Flask ===
const char* flaskServerHost = "192.168.100.21";
const int flaskServerPort = 5000;

String color, batteryLevel, data;
String apiEndpoint = "";

unsigned long previousMillis = 0;
const long interval = 2000;  // Intervalle d’envoi

void setup() {
  Serial.begin(9600);
  delay(100);
  //Serial.println("ESP démarrage...");

  WiFi.begin(ssid, password);
  //Serial.print("Connexion au Wi-Fi");

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 50) {
    delay(500);
    //Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected");
  }
  else {
    Serial.println(" X not Connected"); 
  }
}

void loop() {
  if (!Serial.available()) return;
  if (Serial.available()) {
		delay(100);
    data = Serial.readStringUntil('\n');
    Serial.println("D");

    // --- Envoyer la requête HTTP GET ---
    WiFiClient client;
    HTTPClient http;
    // Traitement de la donnée
    int sep = data.indexOf('#');
    if (sep > 0) {
      color = data.substring(0, sep);
      batteryLevel = data.substring(sep + 1);

      // Construire l'URL complète
      apiEndpoint = "/api/set?color=" + color + "&battery=" + String(batteryLevel);

      String serverPath = "http://" + String(flaskServerHost) + ":" + String(flaskServerPort) + String(apiEndpoint);
      //Serial.println(serverPath);
      if (http.begin(client, serverPath)) {  // HTTP
        int httpCode = http.GET();
        // httpCode will be negative on error
        if (httpCode > 0) {
          Serial.printf("code: %d\n", httpCode);   //Serial.print(httpCode);
          delay(50);    Serial.flush();
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = http.getString();
          }
        }
      } 
      else {}
      http.end();
    } 
    else {//Serial.printf("[HTTP] Unable to connect\n");
    }
  }
}

