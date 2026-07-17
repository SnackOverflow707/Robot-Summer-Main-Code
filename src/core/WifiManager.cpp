#include "core/WifiManager.h"
#include "tape_logic/TapeFollower.h"
#include "tape_logic/SideSensors.h"

WifiManager::WifiManager(
    const char* ssid,
    const char* password,
    MecanumDrive& drive
)
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

    // Configure the HTTP routes once.
    setupRoutes();
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

    const bool started = WiFi.softAP(
        "Robot-Control",
        "robot1234"
    );

    if (!started)
    {
        WiFi.mode(WIFI_OFF);
        return false;
    }

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

    // Stop the robot before removing remote control.
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
    _server.on("/startTape", HTTP_GET, [this]()
{
    setTapeFollowing(true);

    _server.send(
        200,
        "text/plain",
        "Tape follower enabled"
    );
});

_server.on("/stopTape", HTTP_GET, [this]()
{
    setTapeFollowing(false);

    _server.send(
        200,
        "text/plain",
        "Tape follower disabled"
    );
});
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
        _server.send(
            200,
            "text/plain",
            "Rotate Counterclockwise"
        );
    });

    _server.on("/rotateBackCW", HTTP_GET, [this]()
    {
        _drive.rotateClockwiseBackAxis(150);
        _server.send(
            200,
            "text/plain",
            "Rotate Clockwise Around Back Axis"
        );
    });

    _server.on("/rotateBackCCW", HTTP_GET, [this]()
    {
        _drive.rotateCounterClockwiseBackAxis(150);
        _server.send(
            200,
            "text/plain",
            "Rotate Counterclockwise Around Back Axis"
        );
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
        _server.send(200, "text/plain", "Stopped");
    });

    /*
     * Return tape-follower data as JSON.
     */
    _server.on("/status", HTTP_GET, [this]()
    {
        const TapeFollowerStatus status =
            getTapeFollowerStatus();
        const SideSensorStatus sideStatus = getSideSensorStatus();

        String json;
        json.reserve(300);

        json += "{";

        json += "\"leftVoltage\":";
        json += String(status.leftVoltage, 3);

        json += ",\"rightVoltage\":";
        json += String(status.rightVoltage, 3);

        json += ",\"leftWhite\":";
        json += (status.leftWhite ? "true" : "false");

        json += ",\"rightWhite\":";
        json += (status.rightWhite ? "true" : "false");

        json += ",\"error\":";
        json += String(status.error, 2);

        json += ",\"pidOutput\":";
        json += String(status.pidOutput, 2);

        json += ",\"integral\":";
        json += String(status.integral, 2);

        json += ",\"derivative\":";
        json += String(status.derivative, 2);

        json += ",\"kp\":";
        json += String(status.kp, 2);

        json += ",\"ki\":";
        json += String(status.ki, 2);

        json += ",\"kd\":";
        json += String(status.kd, 2);

        json += ",\"sideSensorVoltage\":";
        json += String(sideStatus.sensorVoltage, 3);

        json += ",\"sideOnTape\":";
        json += (sideStatus.onTape ? "true" : "false");

        json += "}";

        _server.send(
            200,
            "application/json",
            json
        );
    });

    /*
     * Example:
     * /setPID?kp=20&ki=0&kd=5
     */
    _server.on("/setPID", HTTP_GET, [this]()
    {
        if (
            !_server.hasArg("kp") ||
            !_server.hasArg("ki") ||
            !_server.hasArg("kd")
        )
        {
            _server.send(
                400,
                "text/plain",
                "Missing kp, ki, or kd"
            );

            return;
        }

        const float kp = _server.arg("kp").toFloat();
        const float ki = _server.arg("ki").toFloat();
        const float kd = _server.arg("kd").toFloat();

        setTapePID(kp, ki, kd);

        _server.send(
            200,
            "text/plain",
            "PID values updated"
        );
    });

    _server.on("/resetPID", HTTP_GET, [this]()
    {
        resetTapePID();

        _server.send(
            200,
            "text/plain",
            "PID state reset"
        );
    });

    _server.onNotFound([this]()
    {
        _server.send(
            404,
            "text/plain",
            "Page or command not found"
        );
    });
}

void WifiManager::showControlPage()
{
    const char* page = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">

    <meta
        name="viewport"
        content="width=device-width, initial-scale=1"
    >

    <title>Robot Control</title>

    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 750px;
            margin: auto;
            padding: 16px;
            background: #eeeeee;
        }

        .panel {
            background: white;
            padding: 16px;
            margin-bottom: 16px;
            border-radius: 10px;
        }

        .sensor-container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }

        .sensor {
            padding: 14px;
            border: 2px solid #999999;
            border-radius: 8px;
        }

        .on-tape {
            background: #c8f7c8;
        }

        .off-tape {
            background: #f7c8c8;
        }

        button {
            min-width: 130px;
            min-height: 45px;
            margin: 5px;
            font-size: 15px;
        }

        .stop-button {
            min-width: 200px;
            font-size: 20px;
            font-weight: bold;
        }

        input {
            width: 75px;
            padding: 7px;
            margin: 5px;
        }

        .value {
            font-weight: bold;
        }

        #connectionStatus {
            font-weight: bold;
        }
    </style>
</head>

<body>

<h1>Robot Control</h1>

<div class="panel">
    <h2>Tape Sensors</h2>

    <div class="sensor-container">
        <div id="leftSensorBox" class="sensor">
            <h3>Left Sensor</h3>

            <p>
                Voltage:
                <span id="leftVoltage" class="value">--</span> V
            </p>

            <p id="leftTapeStatus" class="value">
                Waiting...
            </p>
        </div>

        <div id="rightSensorBox" class="sensor">
            <h3>Right Sensor</h3>

            <p>
                Voltage:
                <span id="rightVoltage" class="value">--</span> V
            </p>

            <p id="rightTapeStatus" class="value">
                Waiting...
            </p>
        </div>
    </div>
</div>

<div class="panel">
    <h2>Side Sensor</h2>
    <div id="sideSensorBox" class="sensor">
        <p>
            Voltage:
            <span id="sideSensorVoltage" class="value">--</span> V
        </p>
        <p id="sideTapeStatus" class="value">Waiting...</p>
    </div>
</div>

<div class="panel">
    <h2>PID Information</h2>

    <p>
        Error:
        <span id="error" class="value">--</span>
    </p>

    <p>
        PID output:
        <span id="pidOutput" class="value">--</span>
    </p>

    <p>
        Integral:
        <span id="integral" class="value">--</span>
    </p>

    <p>
        Derivative:
        <span id="derivative" class="value">--</span>
    </p>

    <h3>PID Settings</h3>

    <label>
        Kp:
        <input
            id="kp"
            type="number"
            step="0.1"
        >
    </label>

    <label>
        Ki:
        <input
            id="ki"
            type="number"
            step="0.1"
        >
    </label>

    <label>
        Kd:
        <input
            id="kd"
            type="number"
            step="0.1"
        >
    </label>

    <br>

    <button onclick="updatePID()">
        Update PID
    </button>

    <button onclick="sendCommand('/resetPID')">
        Reset PID State
    </button>
</div>
<div class="panel">
    <h2>Tape Follower</h2>

    <button onclick="sendCommand('/startTape')">
        Start Tape Following
    </button>

    <button onclick="sendCommand('/stopTape')">
        Stop Tape Following
    </button>
</div>
<div class="panel">
    <h2>Mecanum Drive Control</h2>

    <button onclick="sendCommand('/forward')">
        Forward
    </button>

    <button onclick="sendCommand('/backward')">
        Backward
    </button>

    <br>

    <button onclick="sendCommand('/strafeLeft')">
        Strafe Left
    </button>

    <button onclick="sendCommand('/strafeRight')">
        Strafe Right
    </button>

    <br>

    <button onclick="sendCommand('/rotateCW')">
        Rotate CW
    </button>

    <button onclick="sendCommand('/rotateCCW')">
        Rotate CCW
    </button>

    <br>

    <button onclick="sendCommand('/rotateBackCW')">
        Back Axis CW
    </button>

    <button onclick="sendCommand('/rotateBackCCW')">
        Back Axis CCW
    </button>

    <br><br>

    <button onclick="sendCommand('/frontLeft')">
        Front Left
    </button>

    <button onclick="sendCommand('/frontRight')">
        Front Right
    </button>

    <br>

    <button onclick="sendCommand('/backLeft')">
        Back Left
    </button>

    <button onclick="sendCommand('/backRight')">
        Back Right
    </button>

    <br><br>

    <button
        class="stop-button"
        onclick="sendCommand('/stop')"
    >
        STOP
    </button>

    <p>
        Command:
        <span id="commandStatus" class="value">Ready</span>
    </p>
</div>

<p id="connectionStatus">
    Connecting to ESP32...
</p>

<script>
let pidInputsInitialized = false;

function setSensorDisplay(
    boxId,
    textId,
    sensorIsWhite
) {
    const box = document.getElementById(boxId);
    const text = document.getElementById(textId);

    box.classList.remove("on-tape", "off-tape");

    /*
     * Your TapeFollower code considers:
     *     voltage below threshold = white
     *
     * This assumes the tape is dark.
     */
    if (sensorIsWhite) {
        text.textContent = "OFF TAPE";
        box.classList.add("off-tape");
    } else {
        text.textContent = "ON TAPE";
        box.classList.add("on-tape");
    }
}

async function updateStatus()
{
    try {
        const response = await fetch(
            "/status",
            { cache: "no-store" }
        );

        if (!response.ok) {
            throw new Error("Status request failed");
        }

        const data = await response.json();

        document.getElementById(
            "leftVoltage"
        ).textContent = Number(data.leftVoltage).toFixed(3);

        document.getElementById(
            "rightVoltage"
        ).textContent = Number(data.rightVoltage).toFixed(3);

        setSensorDisplay(
            "leftSensorBox",
            "leftTapeStatus",
            data.leftWhite
        );

        setSensorDisplay(
            "rightSensorBox",
            "rightTapeStatus",
            data.rightWhite
        );

        document.getElementById(
            "sideSensorVoltage"
        ).textContent = Number(data.sideSensorVoltage).toFixed(3);

        setSensorDisplay(
            "sideSensorBox",
            "sideTapeStatus",
            data.sideOnTape
        );

        document.getElementById(
            "error"
        ).textContent = Number(data.error).toFixed(2);

        document.getElementById(
            "pidOutput"
        ).textContent = Number(data.pidOutput).toFixed(2);

        document.getElementById(
            "integral"
        ).textContent = Number(data.integral).toFixed(2);

        document.getElementById(
            "derivative"
        ).textContent = Number(data.derivative).toFixed(2);

        /*
         * Only copy PID values into the inputs once.
         * Otherwise the webpage would overwrite them while
         * you are typing.
         */
        if (!pidInputsInitialized) {
            document.getElementById("kp").value = data.kp;
            document.getElementById("ki").value = data.ki;
            document.getElementById("kd").value = data.kd;

            pidInputsInitialized = true;
        }

        document.getElementById(
            "connectionStatus"
        ).textContent = "ESP32 connected";
    }
    catch (error) {
        document.getElementById(
            "connectionStatus"
        ).textContent = "ESP32 connection lost";
    }
}

async function sendCommand(command)
{
    try {
        const response = await fetch(command);
        const text = await response.text();

        document.getElementById(
            "commandStatus"
        ).textContent = text;
    }
    catch (error) {
        document.getElementById(
            "commandStatus"
        ).textContent = "Command failed";
    }
}

async function updatePID()
{
    const kp = document.getElementById("kp").value;
    const ki = document.getElementById("ki").value;
    const kd = document.getElementById("kd").value;

    if (kp === "" || ki === "" || kd === "") {
        document.getElementById(
            "commandStatus"
        ).textContent = "Enter all three PID values";

        return;
    }

    const command =
        "/setPID?kp=" + encodeURIComponent(kp) +
        "&ki=" + encodeURIComponent(ki) +
        "&kd=" + encodeURIComponent(kd);

    await sendCommand(command);
}

// Request updated values five times per second.
setInterval(updateStatus, 200);

updateStatus();
</script>

</body>
</html>
)rawliteral";

    _server.send(
        200,
        "text/html",
        page
    );
}