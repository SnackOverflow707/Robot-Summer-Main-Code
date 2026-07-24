#pragma once
#include "Arduino.h"
#include "esp32-hal-ledc.h"

/*
 * ArmServo.h — lightweight servo library for ESP32
 * ─────────────────────────────────────────────────
 * Drives servos directly via ESP32 LEDC PWM peripheral.
 * No external library required beyond the ESP32 Arduino core.
 *
 * USAGE:
 *   ArmServo shoulder(13, 0);   // pin 13, LEDC channel 0
 *   shoulder.attach();
 *   shoulder.write(90);         // move to 90°
 *   shoulder.writeMicroseconds(1500);
 *   shoulder.moveTo(45, 60);    // move to 45° at 60°/sec
 *   int a = shoulder.read();    // returns last commanded angle
 *
 * LEDC CHANNELS:
 *   ESP32 has 16 LEDC channels (0–15). Assign one unique
 *   channel per servo — no two servos can share a channel.
 *
 * PULSE WIDTH DEFAULTS:
 *   MIN_PULSE_US →  0°
 *   MAX_PULSE_US → 180°
 */

// ── Default pulse range (µs) for MG996R 
#define ARM_SERVO_MIN_US   1000
#define ARM_SERVO_MAX_US  2000
#define ARM_SERVO_FREQ_HZ   50   // standard 50 Hz (20 ms period)
#define ARM_SERVO_TIMER_BITS 12  

#define MIN_ANGLE 0 
#define MAX_ANGLE 270 

class ArmServo {
public:
    // ── Constructor ─────────────────────────────────────
    // pin      : GPIO pin connected to servo signal wire
    // channel  : LEDC channel (0–15), unique per servo
    // minUs    : pulse width for 0°   (default 1000 µs)
    // maxUs    : pulse width for 180° (default 2000 µs)
ArmServo(int pin,
         int channel,
         int minUs = ARM_SERVO_MIN_US,
         int maxUs = ARM_SERVO_MAX_US,
         int minAngle = MIN_ANGLE,      // ← add these
         int maxAngle = MAX_ANGLE)
    : _pin(pin),
      _channel(channel),
      _minUs(minUs),
      _maxUs(maxUs),
      _currentAngle(-1),
      _attached(false),
      _minAngle(minAngle),       // ← initialize directly
      _maxAngle(maxAngle),
      _inputMinAngle(minAngle),  // ← store as the allowed range for setLimits
      _inputMaxAngle(maxAngle)
    {}

    // ── attach() ────────────────────────────────────────
    // Call once in setup(). Configures the LEDC channel
    // and attaches it to the GPIO pin.
    void attach() {
        ledcSetup(_channel, ARM_SERVO_FREQ_HZ, ARM_SERVO_TIMER_BITS);
        ledcAttachPin(_pin, _channel);
        _attached = true;
    }

    /*int angleToBits(int angle) const {
        return (int)((float)angle / 180.0f * (float)((1 << ARM_SERVO_TIMER_BITS) - 1));
    }

    // ── write(angle) ────────────────────────────────────
    // Move servo to angle (0–180°).
    // Clamps to [minAngle, maxAngle] set via setLimits().
    void manualWrite(int angle) {
        if (!_attached) return;
        angle = constrain(angle, _minAngle, _maxAngle);
        _currentAngle = angle;
        int bits = angleToBits(angle);
        printf("Writing angle: %d, bits: %d\n", angle, bits);
        ledcWrite(_channel, bits);
    }*/ 


        // ── write(angle) ────────────────────────────────────
    // Converts angle → pulse width (µs) → duty cycle ticks.
    void write(int angle) {
        if (!_attached) return;
        angle = constrain(angle, _minAngle, _maxAngle);
        _currentAngle = angle;
 
        uint16_t us = _angleToUs(angle);
        uint32_t ticks = _usToTicks(us);
 
        printf("[ArmServo] pin=%d  angle=%d°  us=%dµs  ticks=%lu  duty=%.1f%%\n",
               _pin, angle, us, ticks, (float)ticks / 4095.0f * 100.0f);
 
        ledcWrite(_channel, ticks);
    }








    // ── writeMicroseconds(us) ───────────────────────────
    // Move servo to a raw pulse width in microseconds.
    // Useful for fine-tuning beyond 0–180° if your servo
    // supports it. Clamps to [_minUs, _maxUs].
    void writeMicroseconds(uint16_t us) {
        if (!_attached) return;
        us = constrain(us, _minUs, _maxUs);
        // Back-calculate angle for read() consistency
        _currentAngle = _usToAngle(us);
        _writeTicks(_usToTicks(us));
    }

    // ── moveTo(targetAngle, degreesPerSec) ──────────────
    // Sweep from current angle to targetAngle at a fixed
    // speed (°/sec). BLOCKING — holds the CPU until done.
    // Use in task sequences, not in a tight loop.
    //
    // Example: moveTo(0, 90)  → sweep to 0° at 90°/sec
    //          moveTo(180, 30) → slow sweep to 180°
    void moveTo(int targetAngle, float degreesPerSec) {
        if (!_attached) return;
        if (_currentAngle < 0) {
            // No known position yet — jump directly
            //manualWrite(targetAngle);
            write(targetAngle);
            return;
        }

        targetAngle = constrain(targetAngle, _minAngle, _maxAngle);
        int startAngle = _currentAngle;
        int delta = targetAngle - startAngle;
        if (delta == 0) return;

        float totalTime_ms = (abs(delta) / degreesPerSec) * 1000.0f;
        float stepDelay_ms = totalTime_ms / abs(delta);

        // Step 1° at a time
        int step = (delta > 0) ? 1 : -1;
        int angle = startAngle;
        while (angle != targetAngle) {
            angle += step;
            write(angle);
            delay((uint32_t)stepDelay_ms);
        }
    }

    // ── read() ──────────────────────────────────────────
    // Returns the last angle commanded via write() or moveTo().
    // Returns -1 if no angle has been set yet this session.
    // NOTE: this is software state — the ESP32 has no way
    // to electrically read back what angle the servo is at.
    int read() const {
        return _currentAngle;
    }

    // ── readMicroseconds() ──────────────────────────────
    // Returns the last commanded pulse width in µs.
    uint16_t readMicroseconds() const {
        if (_currentAngle < 0) return 0;
        return _angleToUs(_currentAngle);
    }

    // ── setLimits(minAngle, maxAngle) ───────────────────
    // Restrict the servo's range. All write() and moveTo()
    // calls will be clamped to [minAngle, maxAngle].
    // Call before attach() or after — order doesn't matter.
    void setLimits(int minAngle, int maxAngle) {
        _minAngle = constrain(minAngle, _inputMinAngle, _inputMaxAngle);
        _maxAngle = constrain(maxAngle, _inputMinAngle, _inputMaxAngle);
    }

    // ── detach() ────────────────────────────────────────
    // Stops PWM output and releases the LEDC channel.
    // Servo will go limp (no holding torque).
    void detach() {
        if (_attached) {
            ledcDetachPin(_pin);
            _attached = false;
        }
    }

    bool isAttached() const { return _attached; }
    uint8_t getPin()     const { return _pin; }
    uint8_t getChannel() const { return _channel; }

private:
    uint8_t  _pin;
    uint8_t  _channel;
    uint16_t _minUs;
    uint16_t _maxUs;
    int      _currentAngle;
    bool     _attached;
    int _minAngle; 
    int _maxAngle; 
     int      _inputMinAngle;   
    int      _inputMaxAngle;

    // ── angle (°) → pulse width (µs) ───────────────────
    uint16_t _angleToUs(int angle) const
{
    angle = constrain(angle, _inputMinAngle, _inputMaxAngle);

    return static_cast<uint16_t>(
        map(
            angle,
            _inputMinAngle,
            _inputMaxAngle,
            _minUs,
            _maxUs
        )
    );
}

    // ── pulse width (µs) → angle (°) ───────────────────
    int _usToAngle(uint16_t us) const
{
    us = constrain(us, _minUs, _maxUs);

    return static_cast<int>(
        map(
            us,
            _minUs,
            _maxUs,
            _inputMinAngle,
            _inputMaxAngle
        )
    );
}

    // ── pulse width (µs) → LEDC duty ticks ─────────────
    // Period = 1/50Hz = 20,000 µs
    // Ticks  = (us / 20000) × 65535
uint32_t _usToTicks(uint16_t us) const {
    // PWM period in microseconds
    constexpr uint32_t PERIOD_US = 1000000UL / ARM_SERVO_FREQ_HZ;
    // Maximum tick value for the configured timer resolution
    constexpr uint32_t MAX_TICKS = (1UL << ARM_SERVO_TIMER_BITS) - 1;
    // Convert pulse width (us) to duty ticks
    return ((uint32_t)us * MAX_TICKS) / PERIOD_US;
}

    void _writeTicks(uint32_t ticks) {
        ledcWrite(_channel, ticks);
    }
};
