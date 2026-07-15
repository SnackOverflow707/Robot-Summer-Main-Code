#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include "actuators/MecanumDrive.h"

class WifiManager {
public:
    WifiManager(
        const char* ssid,
        const char* password,
        MecanumDrive& drive
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

    bool _enabled;

    void setupRoutes();
    void showControlPage();
};