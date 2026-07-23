#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include "actuators/MecanumDrive.h"

class ArmController2;

class WifiManager {
public:
    WifiManager(
        const char* ssid,
        const char* password,
        MecanumDrive& drive,
        ArmController2& arm
    );

    void begin();
    void update();

    bool enable();
    void disable();

    bool isEnabled() const;

private:
    const char* _ssid;
    const char* _password;

    WebServer _server;
    MecanumDrive& _drive;
    ArmController2& _arm;

    bool _enabled;

    void setupRoutes();
    void showControlPage();
};