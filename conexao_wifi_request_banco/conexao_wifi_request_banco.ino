#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "login";
const char* password = "password";

const char* apiUrl = "https://joaopedrogalera.pythonanywhere.com:80/machineSync";  // Replace with your API endpoint

int cont = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (cont == 0) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;

      Serial.print("Connecting to server: ");
      Serial.println(apiUrl);

      http.begin(client, apiUrl);

      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("teste");

      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);

      String response = http.getString();

      if (httpResponseCode > 0) {
        Serial.println("Response message:");
        Serial.println(response);
      } else {
        Serial.println("Error sending request");
      }
        
      // Free resources
      http.end();

    } else {
      Serial.println("WiFi disconnected!");
    }
  }
  cont = 1;
}
