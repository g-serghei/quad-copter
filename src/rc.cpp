#include <Arduino.h>

#include "rc.h"
#include "log.h"

namespace Rc
{
    const char *SSID = "CopterAP";
    const char *PASSWORD = "123";

    const int ESC_PIN1 = 12;
    const int ESC_PIN2 = 13;
    const int ESC_PIN3 = 14;
    const int ESC_PIN4 = 27;

    Servo ESC1;
    Servo ESC2;
    Servo ESC3;
    Servo ESC4;

    AsyncWebServer server(80);
    AsyncWebSocket ws("/ws");

    int throttle = 0;
    int pitch = 0;
    int roll = 0;
    int yaw = 0;

    int motorFrontRight = 0;
    int motorRearRight = 0;
    int motorRearLeft = 0;
    int motorFrontLeft = 0;

    void init()
    {
        Log::info("Configuring access point...");

        WiFi.softAP(SSID, PASSWORD);
        IPAddress myIP = WiFi.softAPIP();
        Log::info("AP IP address: ");
        Log::info(myIP);

        ws.onEvent(onEvent);
        server.addHandler(&ws);

        server.begin();

        ESC1.attach(ESC_PIN1, 1000, 2000);
        ESC2.attach(ESC_PIN2, 1000, 2000);
        ESC3.attach(ESC_PIN3, 1000, 2000);
        ESC4.attach(ESC_PIN4, 1000, 2000);

        pinMode(LED_BUILTIN, OUTPUT);
    }

    void process()
    {
        ws.cleanupClients();

        motorFrontRight = throttle - roll - pitch - yaw;
        motorRearRight = throttle - roll + pitch + yaw;
        motorRearLeft = throttle + roll + pitch - yaw;
        motorFrontLeft = throttle + roll - pitch + yaw;

        ESC1.write(motorFrontRight);
        ESC2.write(motorRearRight);
        ESC3.write(motorRearLeft);
        ESC4.write(motorFrontLeft);
    }

    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;

        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
        {
            data[len] = 0;

            StaticJsonDocument<200> doc;

            char *json = (char *)data;

            DeserializationError error = deserializeJson(doc, json);

            if (error)
            {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            throttle = doc["throttle"];
            pitch = doc["pitch"];
            roll = doc["roll"];
            yaw = doc["yaw"];
        }
    }

    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
    {
        switch (type)
        {
        case WS_EVT_CONNECT:
            digitalWrite(LED_BUILTIN, HIGH);
            //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            emergencyStop();
            //Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            emergencyStop();
            break;
        }
    }

    void emergencyStop()
    {
        digitalWrite(LED_BUILTIN, LOW);

        throttle = 0;
        pitch = 0;
        roll = 0;
        yaw = 0;
    }

}