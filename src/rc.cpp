#include <Arduino.h>

#include "GyverPID.h"

#include "rc.h"
#include "log.h"
#include "imu.h"

namespace Rc
{
    const char *SSID = "CopterAP";
    const char *PASSWORD = "123";

    const int ESC_PIN1 = 26;
    const int ESC_PIN2 = 25;
    const int ESC_PIN3 = 32;
    const int ESC_PIN4 = 33;

    const float MIN_ANGLE = 1;  // deg
    const float MAX_ANGLE = 25.0; // deg
    const int MAX_MOTOR_VALUE = 150;

    Servo ESC1;
    Servo ESC2;
    Servo ESC3;
    Servo ESC4;

    const float KP = 2.5;
    const float KI = 0;
    const float KD = 2;

    GyverPID regulatorPitch(KP, KI, KD);
    GyverPID regulatorRoll(KP, KI, KD);
    GyverPID regulatorYaw(KP, KI, KD);
    int16_t lastRegulatorUpdate = millis();

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

        regulatorPitch.setLimits(-1 * MAX_MOTOR_VALUE, MAX_MOTOR_VALUE);
        regulatorRoll.setLimits(-1 * MAX_MOTOR_VALUE, MAX_MOTOR_VALUE);
    }

    void process()
    {
        ws.cleanupClients();

        AxisType angles = Imu::getDegAngles();

        angles.x = min(angles.x, MAX_ANGLE);
        angles.y = min(angles.y, MAX_ANGLE);

        regulatorPitch.setpoint = pitch;
        regulatorRoll.setpoint = roll;

        if (throttle > 0)
        {
            regulatorPitch.input = abs(angles.y) > MIN_ANGLE ? angles.y : 0;
            regulatorRoll.input = abs(angles.x) > MIN_ANGLE ? angles.x : 0;
        }
        else
        {
            regulatorPitch.input = 0;
            regulatorRoll.input = 0;
        }

        int16_t dt = millis() - lastRegulatorUpdate;
        regulatorPitch.setDt(dt);
        regulatorPitch.getResult();

        regulatorRoll.setDt(dt);
        regulatorRoll.getResult();

        lastRegulatorUpdate = millis();

        float M_RATE = 0.8;

        motorFrontRight = abs(throttle - (regulatorRoll.output * M_RATE) - (regulatorPitch.output * M_RATE) - yaw);
        motorRearRight = abs(throttle - (regulatorRoll.output * M_RATE) + (regulatorPitch.output * M_RATE) + yaw);
        motorRearLeft = abs(throttle + (regulatorRoll.output * M_RATE) + (regulatorPitch.output * M_RATE) - yaw);
        motorFrontLeft = abs(throttle + (regulatorRoll.output * M_RATE) - (regulatorPitch.output * M_RATE) + yaw);

        Serial.print("Setpoint:");
        Serial.print(regulatorRoll.setpoint);
        Serial.print(",");

        Serial.print("InputX:");
        Serial.print(regulatorRoll.input);
        Serial.print(",");

        Serial.print("Input Y:");
        Serial.print(regulatorPitch.input);
        Serial.print(",");

        // Serial.print("motorFrontRight:");
        // Serial.print(motorFrontRight);
        // Serial.print(",");

        // Serial.print("motorRearRight:");
        // Serial.print(motorRearRight);
        // Serial.print(",");

        // Serial.print("motorRearLeft:");
        // Serial.print(motorRearLeft);
        // Serial.print(",");

        // Serial.print("motorFrontLeft:");
        // Serial.println(motorFrontLeft);

        Serial.print("OutputX:");
        Serial.print(regulatorRoll.output);
        Serial.print(",");

        Serial.print("Output Y:");
        Serial.println(regulatorPitch.output);

        // motorFrontRight = throttle - roll - pitch - yaw;
        // motorRearRight = throttle - roll + pitch + yaw;
        // motorRearLeft = throttle + roll + pitch - yaw;
        // motorFrontLeft = throttle + roll - pitch + yaw;

        motorFrontRight = min(MAX_MOTOR_VALUE, motorFrontRight);
        motorRearRight = min(MAX_MOTOR_VALUE, motorRearRight);
        motorRearLeft = min(MAX_MOTOR_VALUE, motorRearLeft);
        motorFrontLeft = min(MAX_MOTOR_VALUE, motorFrontLeft);

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
                Log::warning("deserializeJson() failed");
                return;
            }

            throttle = map(doc["throttle"], 0, 180, 0, MAX_MOTOR_VALUE);
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