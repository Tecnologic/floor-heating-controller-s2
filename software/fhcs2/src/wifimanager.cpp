#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "filesystem.h"
namespace wifimanager
{
  // Search for parameter in HTTP POST request
  const char *TAG_SSID = "ssid";
  const char *TAG_PASS = "pass";
  const char *TAG_IP = "ip";
  const char *TAG_GATE = "gateway";
  const char *TAG_HOST = "hostname";

  // Allocate the JSON document
  //
  // Inside the brackets, 200 is the capacity of the memory pool in bytes.
  // Don't forget to change this value to match your JSON document.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<1024> doc;

  // Create AsyncWebServer object on port 80
  WebServer server(80);

  // DNS Server for Captiv Portal
  const byte DNS_PORT = 53;
  IPAddress apIP(8, 8, 4, 4); // The default android DNS
  DNSServer dnsServer;

  // Variables to save values from HTML form
  String ssid;
  String pass;
  String ip;
  String gateway;
  String hostname;

  // File paths to save input values permanently
  const char *configJasonPath = "/config.json";

  IPAddress localIP;

  // Set your Gateway IP address
  IPAddress localGateway;
  IPAddress subnet(255, 255, 0, 0);

  // Timer variables
  std::uint32_t previousMillis = 0;
  constexpr std::uint32_t interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

  bool restart = false;

  bool softAp = false;

  void handleNotFound(void)
  {
    File dataFile = SD.open(path.c_str());
    if (dataFile.isDirectory())
    {
      path += "/index.htm";
      dataType = "text/html";
      dataFile = SD.open(path.c_str());
    }

    if (!dataFile)
    {
      return false;
    }

    if (server.streamFile(dataFile, dataType) != dataFile.size())
    {
      DBG_OUTPUT_PORT.println("Sent less data than expected!");
    }

    dataFile.close();
  }

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
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, filesystem::readFile(configJasonPath));
    // Load values saved in LittleFS
    ssid = doc[TAG_SSID];
    pass = doc[TAG_PASS];
    ip = doc[TAG_IP];
    gateway = doc[TAG_GATE];
    hostname = doc[TAG_HOST];
    Serial.println(ssid);
    Serial.println(pass);
    Serial.println(ip);
    Serial.println(gateway);
    Serial.println(hostname);

    if (initWiFi())
    {
      if (MDNS.begin(hostname))
      {
        MDNS.addService("http", "tcp", 80);
        Serial.println("MDNS responder started");
        Serial.print("You can now connect to http://");
        Serial.print(hostname);
        Serial.println(".local");
      }

      // // Web Server Root URL
      server.on("/", HTTP_GET, []()
                { server.send(200, "text/html", filesystem::readFile("/index.html")); });

      server.serveStatic("/", LittleFS, "/");

      server.begin();
    }
    else
    {
      softAp = true;
      // Connect to Wi-Fi network with SSID and password
      Serial.println("Setting AP (Access Point)");
      // NULL sets an open Access Point
      WiFi.softAP("ESP-WIFI-MANAGER", NULL);
      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

      // if DNSServer is started with "*" for domain name, it will reply with
      // provided IP to all DNS request
      dnsServer.start(DNS_PORT, "*", apIP);

      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);

      // // Web Server Root URL
      server.on("/", HTTP_GET, []()
                { server.send(200, "text/html", filesystem::readFile("/index.html")); });

      server.serveStatic("/", LittleFS, "/");

      server.begin();
    }
  }

  void loop()
  {
    if (softAp)
    {
      dnsServer.processNextRequest();
    }
    server.handleClient();
  }
} /* namespace wifimanager */