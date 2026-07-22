#ifndef ARM_WIFI_MANAGER_H
#define ARM_WIFI_MANAGER_H

#include <WebServer.h>
#include <WiFi.h>
#include "ArmController2.h"



class ArmWifiManager
{
public:
    ArmWifiManager(
        const char* ssid,
        const char* password,
        ArmController2& arm
    );

    void begin();
    void update();

    bool enable();
    void disable();

    bool isEnabled() const;

private:
    void setupRoutes();
    void showControlPage();

    const char* _ssid;
    const char* _password;

    WebServer _server;
    ArmController2& _arm;

    bool _enabled;
};

#endif