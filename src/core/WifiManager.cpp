#include "core/WifiManager.h"

WifiManager::WifiManager(
    const char* ssid,
    const char* password,
    MecanumDrive& drive)
    : _ssid(ssid),
      _password(password),
      _server(80),
      _drive(drive),
      _enabled(false)
{
}

void WifiManager::begin()
{
    WiFi.mode(WIFI_OFF);
}

void WifiManager::update()
{
    if (_enabled)
    {
        _server.handleClient();
    }
}

bool WifiManager::enable()
{
    if (_enabled)
    {
        return true;
    }

    WiFi.mode(WIFI_AP);

    bool started = WiFi.softAP(
        "Robot-Control",
        "robot1234"
    );

    if (!started)
    {
        return false;
    }

    setupRoutes();
    _server.begin();

    _enabled = true;

    return true;
}

void WifiManager::disable()
{
    if (!_enabled)
    {
        return;
    }

    _drive.stop();
    _server.stop();

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    _enabled = false;
}

bool WifiManager::isEnabled() const
{
    return _enabled;
}

void WifiManager::setupRoutes()
{
    _server.on("/", HTTP_GET, [this]()
    {
        showControlPage();
    });

    _server.on("/forward", HTTP_GET, [this]()
    {
        _drive.forward(150);
        _server.send(200, "text/plain", "Forward");
    });

    _server.on("/backward", HTTP_GET, [this]()
    {
        _drive.backward(150);
        _server.send(200, "text/plain", "Backward");
    });

    _server.on("/strafeRight", HTTP_GET, [this]()
    {
        _drive.strafeRight(150);
        _server.send(200, "text/plain", "Strafe Right");
    });

    _server.on("/strafeLeft", HTTP_GET, [this]()
    {
        _drive.strafeLeft(150);
        _server.send(200, "text/plain", "Strafe Left");
    });

    _server.on("/rotateCW", HTTP_GET, [this]()
    {
        _drive.rotateClockwise(150);
        _server.send(200, "text/plain", "Rotate Clockwise");
    });

    _server.on("/rotateCCW", HTTP_GET, [this]()
    {
        _drive.rotateCounterClockwise(150);
        _server.send(200, "text/plain", "Rotate Counter Clockwise");
    });

    _server.on("/rotateBackCW", HTTP_GET, [this]()
    {
        _drive.rotateClockwiseBackAxis(150);
        _server.send(200, "text/plain", "Rotate CW Back Axis");
    });

    _server.on("/rotateBackCCW", HTTP_GET, [this]()
    {
        _drive.rotateCounterClockwiseBackAxis(150);
        _server.send(200, "text/plain", "Rotate CCW Back Axis");
    });

    _server.on("/frontLeft", HTTP_GET, [this]()
    {
        _drive.frontLeftMotor(150);
        _server.send(200, "text/plain", "Front Left Motor");
    });

    _server.on("/frontRight", HTTP_GET, [this]()
    {
        _drive.frontRightMotor(150);
        _server.send(200, "text/plain", "Front Right Motor");
    });

    _server.on("/backLeft", HTTP_GET, [this]()
    {
        _drive.backLeftMotor(150);
        _server.send(200, "text/plain", "Back Left Motor");
    });

    _server.on("/backRight", HTTP_GET, [this]()
    {
        _drive.backRightMotor(150);
        _server.send(200, "text/plain", "Back Right Motor");
    });

    _server.on("/stop", HTTP_GET, [this]()
    {
        _drive.stop();
        _server.send(200, "text/plain", "STOP");
    });
}

void WifiManager::showControlPage()
{
    const char* page = R"rawliteral(
<!DOCTYPE html>
<html>
<body>

<h2>Mecanum Drive Control</h2>

<button onclick="send('/forward')">Forward</button>
<button onclick="send('/backward')">Backward</button>

<br><br>

<button onclick="send('/strafeLeft')">Strafe Left</button>
<button onclick="send('/strafeRight')">Strafe Right</button>

<br><br>

<button onclick="send('/rotateCW')">Rotate CW</button>
<button onclick="send('/rotateCCW')">Rotate CCW</button>

<br><br>

<button onclick="send('/rotateBackCW')">Rotate Back CW</button>
<button onclick="send('/rotateBackCCW')">Rotate Back CCW</button>

<br><br>

<button onclick="send('/frontLeft')">Front Left</button>
<button onclick="send('/frontRight')">Front Right</button>

<br><br>

<button onclick="send('/backLeft')">Back Left</button>
<button onclick="send('/backRight')">Back Right</button>

<br><br>

<button onclick="send('/stop')">STOP</button>

<p id="status"></p>

<script>
function send(command)
{
    fetch(command)
    .then(response => response.text())
    .then(text =>
    {
        document.getElementById("status").innerHTML = text;
    });
}
</script>

</body>
</html>
)rawliteral";

    _server.send(200, "text/html", page);
}