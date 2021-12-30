#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

namespace Rc
{
    void init();
    void process();
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void emergencyStop();
}