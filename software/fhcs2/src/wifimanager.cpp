#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include "ota.h"
#include "filesystem.h"
namespace wifimanager
{
  // Search for parameter in Json
  const char *TAG_SSID = "ssid";
  const char *TAG_PASS = "pass";
  const char *TAG_IP = "ip";
  const char *TAG_GATE = "gateway";
  const char *TAG_DNS = "dns";
  const char *TAG_SUBNET = "subnet";
  const char *TAG_HOST = "hostname";

  // Allocate the JSON document
  //
  // Inside the brackets, 200 is the capacity of the memory pool in bytes.
  // Don't forget to change this value to match your JSON document.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<1024> doc;
  char jsonBuffer[1024];

  // Create AsyncWebServer object on port 80
  WebServer server(80);

  // DNS Server for Captive Portal
  const byte DNS_PORT = 53;
  IPAddress apIP(8, 8, 4, 4); // The default android DNS
  DNSServer dnsServer;

  // Variables to save values from HTML form
  const char *ssid;
  const char *pass;
  IPAddress ip;
  IPAddress gateway;
  IPAddress dns;
  IPAddress subnet;
  const char *hostname;

  // File paths to save input values permanently
  const char *configJasonPath = "/config.json";

  // restart needed?
  bool restart = false;

  // currently running in AP Mode?
  bool softAp = false;

  // is the IP Address static?
  bool staticIP = false;

  // Initialize WiFi
  bool initWiFi(void)
  {
    if (ssid == "")
    {
      Serial.println("Undefined SSID.");
      return false;
    }

    WiFi.mode(WIFI_STA);

    if (staticIP)
    {
      if (!WiFi.config(ip, gateway, subnet))
      {
        Serial.println("STA Failed to configure");
        return false;
      }
      else
      {
        Serial.println("STA configured to static:");
        Serial.print("     IP: ");
        Serial.println(ip);
        Serial.print("Gateway: ");
        Serial.println(gateway);
        Serial.print(" Subnet: ");
        Serial.println(subnet);
      }
    }

    WiFi.begin(ssid, pass);

    Serial.println("Connecting to WiFi");

    if (WiFi.waitForConnectResult(30000) != WL_CONNECTED)
    {
      return false;
    }

    Serial.println("STA DHCP IP:");
    Serial.println(WiFi.localIP());
    return true;
  }

  void init()
  {
    // Deserialize the JSON document
    String fileContent = filesystem::readFile(configJasonPath);
    Serial.println("JSON File Content:");
    Serial.println(fileContent);
    DeserializationError error = deserializeJson(doc, fileContent.c_str());

    if (DeserializationError::Code::Ok == error)
    {
      // Load values saved in LittleFS
      ssid = doc[TAG_SSID];
      pass = doc[TAG_PASS];
      ip.fromString((const char *)doc[TAG_IP]);
      gateway.fromString((const char *)doc[TAG_GATE]);
      dns.fromString((const char *)doc[TAG_DNS]);
      subnet.fromString((const char *)doc[TAG_SUBNET]);
      hostname = doc[TAG_HOST];

      if (ip.isAny())
      {
        staticIP = true;
      }
    }

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

      ota::init(hostname);

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

  // loop handling of wifi stuff
  void handle()
  {
    if (softAp)
    {
      dnsServer.processNextRequest();
    }
    server.handleClient();
    ota::handle();
  }
} /* namespace wifimanager */