// #include <Arduino.h>

// /*
// Side sensors to detect when the robot is near the tower pieces
// */

// const int FRONT_SENSOR_PIN = 34;
// const int REAR_SENSOR_PIN = 35;

// const float WHITE_THRESHOLD = 1.65; // midpoint between 0V and 3.3V

// const int MAX_ADC_VALUE = 8191; // for esp32 s3 (13-bit ADC)\

// void setupSideSensors() {
//   // configure your side sensor pins here
//   pinMode(FRONT_SENSOR_PIN, INPUT);
//   pinMode(REAR_SENSOR_PIN, INPUT);
// }

// float readSensorVoltage(int pin) {
//   int raw = analogRead(pin);           
//   return raw * (3.3 / MAX_ADC_VALUE);
// }

// void setup() {
//     setupSideSensors();
//     Serial.begin(115200);
// }
// // If the state of the front or the back sensor changes, then we're near the tower pieces
// void loop() {
//     bool frontIsWhite  = readSensorVoltage(FRONT_SENSOR_PIN)  > WHITE_THRESHOLD;
//     bool rearIsWhite = readSensorVoltage(REAR_SENSOR_PIN) > WHITE_THRESHOLD;

//     if (!frontIsWhite || !rearIsWhite) {
//         // SENSOR TRIGGERED!
//         Serial.println("In position to pick up tower pieces.");
//     }

// }