#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include "filesystem.h"
namespace wifimanager
{
  // Create AsyncWebServer object on port 80
  WebServer server(80);

  // Search for parameter in HTTP POST request
  const char *PARAM_INPUT_1 = "ssid";
  const char *PARAM_INPUT_2 = "pass";
  const char *PARAM_INPUT_3 = "ip";
  const char *PARAM_INPUT_4 = "gateway";

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
  std::uint32_t previousMillis = 0;
  constexpr std::uint32_t interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

  bool restart = false;

  // Initialize WiFi
  bool initWiFi()
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
    delay(20000);
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Failed to connect.");
      return false;
    }

    Serial.println(WiFi.localIP());
    return true;
  }

  void init()
  {
    // Load values saved in LittleFS
    ssid = filesystem::readFile(ssidPath);
    pass = filesystem::readFile(passPath);
    ip = filesystem::readFile(ipPath);
    gateway = filesystem::readFile(gatewayPath);
    Serial.println(ssid);
    Serial.println(pass);
    Serial.println(ip);
    Serial.println(gateway);

    if (initWiFi())
    {
      // Route for root / web page
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(LittleFS, "/index.html", "text/html", false); });

      server.serveStatic("/", LittleFS, "/");

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
                { request->send(LittleFS, "/wifimanager.html", "text/html"); });

      server.serveStatic("/", LittleFS, "/");

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
            filesystem::writeFile(ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            filesystem::writeFile(passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            filesystem::writeFile(ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            filesystem::writeFile(gatewayPath, gateway.c_str());
          }
          Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      restart = true;
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip); });
      server.begin();
    }
  }
} /* namespace wifimanager */