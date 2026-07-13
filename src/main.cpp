// #include <Arduino.h>
// #include "actuators/MecanumDrive.h"

// MecanumDrive drive;

// void setup() {
//     drive.begin();
    
// }

// void loop() {
//     // Forward
//     drive.forward(150);
//     delay(1000);
// }

/*
 * main.cpp for tuning PID
 * ──────────────
 * Runs the tape follower live and lets you tune PID gains
 * via Serial Monitor without reflashing.
 *
 * Commands (115200 baud):
 *   p<value>   → set Kp      e.g. "p40.0"
 *   i<value>   → set Ki      e.g. "i0.1"
 *   d<value>   → set Kd      e.g. "d15.0"
 *   b<value>   → set BASE_SPEED   e.g. "b150"
 *   s<value>   → set STRAFE_SPEED e.g. "s80"
 *   r<value>   → set ROTATE_SPEED e.g. "r100"
 *   c<value>   → set CURVE_THRESHOLD e.g. "c5"
 *   start      → begin tape following
 *   stop       → stop motors
 *   params     → print current values
 */

#include <Arduino.h>
#include "actuators/MecanumDrive.h"
#include "tape_logic/TapeFollower.h"

MecanumDrive drive;

// ── Tunable params — exposed so TapeFollower.cpp can read them ────────────
float Kp             = 40.0f;
float Ki             =  0.0f;
float Kd             = 15.0f;
int   BASE_SPEED     = 100;
int   STRAFE_SPEED   =  80;
int   ROTATE_SPEED   = 100;
int   CURVE_THRESHOLD =  5;

bool running = false;

// ── Print all current values ──────────────────────────────────────────────
void printParams() {
    Serial.println("──────────────────────────────");
    Serial.printf("  Kp:              %.2f\n", Kp);
    Serial.printf("  Ki:              %.2f\n", Ki);
    Serial.printf("  Kd:              %.2f\n", Kd);
    Serial.printf("  BASE_SPEED:      %d\n",   BASE_SPEED);
    Serial.printf("  STRAFE_SPEED:    %d\n",   STRAFE_SPEED);
    Serial.printf("  ROTATE_SPEED:    %d\n",   ROTATE_SPEED);
    Serial.printf("  CURVE_THRESHOLD: %d\n",   CURVE_THRESHOLD);
    Serial.printf("  running:         %s\n",   running ? "YES" : "NO");
    Serial.println("──────────────────────────────");
}

// ── Parse and apply Serial command ───────────────────────────────────────
void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "start") {
        running = true;
        Serial.println("[tuner] started");
        return;
    }
    if (cmd == "stop") {
        running = false;
        drive.stop();
        Serial.println("[tuner] stopped");
        return;
    }
    if (cmd == "params") {
        printParams();
        return;
    }

    char  key   = cmd[0];
    float value = cmd.substring(1).toFloat();

    switch (key) {
        case 'p':
            Kp = value;
            Serial.printf("[tuner] Kp = %.2f\n", Kp);
            break;
        case 'i':
            Ki = value;
            Serial.printf("[tuner] Ki = %.2f\n", Ki);
            break;
        case 'd':
            Kd = value;
            Serial.printf("[tuner] Kd = %.2f\n", Kd);
            break;
        case 'b':
            BASE_SPEED = (int)value;
            Serial.printf("[tuner] BASE_SPEED = %d\n", BASE_SPEED);
            break;
        case 's':
            STRAFE_SPEED = (int)value;
            Serial.printf("[tuner] STRAFE_SPEED = %d\n", STRAFE_SPEED);
            break;
        case 'r':
            ROTATE_SPEED = (int)value;
            Serial.printf("[tuner] ROTATE_SPEED = %d\n", ROTATE_SPEED);
            break;
        case 'c':
            CURVE_THRESHOLD = (int)value;
            Serial.printf("[tuner] CURVE_THRESHOLD = %d\n", CURVE_THRESHOLD);
            break;
        default:
            Serial.println("Unknown. Commands: p i d b s r c start stop params");
            break;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(12, INPUT);   // LEFT_SENSOR_PIN
    pinMode(13, INPUT);   // RIGHT_SENSOR_PIN

    drive.begin();

    Serial.println("\n=== PID Tuner ===");
    Serial.println("Commands: p i d b s r c  |  start  stop  params");
    Serial.println("Send 'start' to begin tape following.");
    printParams();
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    // handle incoming serial command
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    // run tape follower if active
    if (running) {
        tapeFollowStep();
    }
}