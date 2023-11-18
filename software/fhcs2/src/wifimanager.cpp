#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "filesystem.h"

namespace wifi
{
  // Create AsyncWebServer object on port 80
  AsyncWebServer server(80);

  // Search for parameter in HTTP POST request
  constexpr char *PARAM_INPUT_1 = "ssid";
  constexpr char *PARAM_INPUT_2 = "pass";
  constexpr char *PARAM_INPUT_3 = "ip";
  constexpr char *PARAM_INPUT_4 = "gateway";

  // Variables to save values from HTML form
  String ssid;
  String pass;
  String ip;
  String gateway;

  // File paths to save input values permanently
  const char *ssidPath = "/ssid.txt";
  const char *passPath = "/pass.txt";
  const char *ipPath = "/ip.txt";
  const char *gatewayPath = "/gateway.txt";

  IPAddress localIP;

  // Set your Gateway IP address
  IPAddress localGateway;
  IPAddress subnet(255, 255, 0, 0);

  // Timer variables
  unsigned long previousMillis = 0;
  const long interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

  // Initialize WiFi
  bool initWiFi(void)
  {
    if (ssid == "" || ip == "")
    {
      Serial.println("Undefined SSID or IP address.");
      return false;
    }

    WiFi.mode(WIFI_STA);
    localIP.fromString(ip.c_str());
    localGateway.fromString(gateway.c_str());

    if (!WiFi.config(localIP, localGateway, subnet))
    {
      Serial.println("STA Failed to configure");
      return false;
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.println("Connecting to WiFi...");

    unsigned long currentMillis = millis();
    previousMillis = currentMillis;

    while (WiFi.status() != WL_CONNECTED)
    {
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval)
      {
        Serial.println("Failed to connect.");
        return false;
      }
    }

    Serial.println(WiFi.localIP());
    return true;
  }

  void init(void)
  {
    // Load values saved in SPIFFS
    ssid = readFile(ssidPath);
    pass = readFile(passPath);
    ip = readFile(ipPath);
    gateway = readFile(gatewayPath);
    Serial.println(ssid);
    Serial.println(pass);
    Serial.println(ip);
    Serial.println(gateway);

    if (initWiFi())
    {
      // Route for root / web page
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/index.html", "text/html", false, processor); });
      server.serveStatic("/", SPIFFS, "/");

      // Route to set GPIO state to HIGH
      server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request)
                {
      digitalWrite(ledPin, HIGH);
      request->send(SPIFFS, "/index.html", "text/html", false, processor); });

      // Route to set GPIO state to LOW
      server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request)
                {
      digitalWrite(ledPin, LOW);
      request->send(SPIFFS, "/index.html", "text/html", false, processor); });
      server.begin();
    }
    else
    {
      // Connect to Wi-Fi network with SSID and password
      Serial.println("Setting AP (Access Point)");
      // NULL sets an open Access Point
      WiFi.softAP("ESP-WIFI-MANAGER", NULL);

      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);

      // Web Server Root URL
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/wifimanager.html", "text/html"); });

      server.serveStatic("/", SPIFFS, "/");

      server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
                {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      ESP.restart(); });
      server.begin();
    }
  }
}