#include "actuators/MotorDriver.h"

#include <Arduino.h>
#include "driver/mcpwm.h"

static constexpr uint32_t PWM_FREQ = 400;
static constexpr int REVERSE_DEAD_TIME_US = 2000;

MotorDriver::MotorDriver(
    int pinA,
    int pinB,
    mcpwm_unit_t unit,
    mcpwm_timer_t timer
)
    : _pinA(pinA),
      _pinB(pinB),
      _unit(unit),
      _timer(timer),
      _lastSpeed(0)
{
}

/*
 * Convert timer 0, 1, or 2 into its corresponding
 * MCPWM A output.
 */
static mcpwm_io_signals_t getSignalA(
    mcpwm_timer_t timer
)
{
    switch (timer)
    {
        case MCPWM_TIMER_0:
            return MCPWM0A;

        case MCPWM_TIMER_1:
            return MCPWM1A;

        case MCPWM_TIMER_2:
            return MCPWM2A;

        default:
            return MCPWM0A;
    }
}

/*
 * Convert timer 0, 1, or 2 into its corresponding
 * MCPWM B output.
 */
static mcpwm_io_signals_t getSignalB(
    mcpwm_timer_t timer
)
{
    switch (timer)
    {
        case MCPWM_TIMER_0:
            return MCPWM0B;

        case MCPWM_TIMER_1:
            return MCPWM1B;

        case MCPWM_TIMER_2:
            return MCPWM2B;

        default:
            return MCPWM0B;
    }
}

void MotorDriver::begin()
{
    /*
     * Route the timer's A and B outputs to the two
     * H-bridge input pins.
     */
    ESP_ERROR_CHECK(
        mcpwm_gpio_init(
            _unit,
            getSignalA(_timer),
            _pinA
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_gpio_init(
            _unit,
            getSignalB(_timer),
            _pinB
        )
    );

    mcpwm_config_t config = {};

    config.frequency = PWM_FREQ;

    // Both outputs start at zero duty.
    config.cmpr_a = 0.0f;
    config.cmpr_b = 0.0f;

    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;

    ESP_ERROR_CHECK(
        mcpwm_init(
            _unit,
            _timer,
            &config
        )
    );

    stop();
}

void MotorDriver::setSpeed(int speed)
{
    speed = constrain(speed, -255, 255);

    const bool reversing =
        (speed > 0 && _lastSpeed < 0) ||
        (speed < 0 && _lastSpeed > 0);

    if (reversing)
    {
        stop();
        delayMicroseconds(REVERSE_DEAD_TIME_US);
    }

    if (speed == 0)
    {
        stop();
        return;
    }

    const float duty =
        100.0f *
        static_cast<float>(abs(speed)) /
        255.0f;

    if (speed > 0)
    {
        /*
         * Forward:
         * A = PWM
         * B = LOW
         */
        ESP_ERROR_CHECK(
            mcpwm_set_signal_low(
                _unit,
                _timer,
                MCPWM_OPR_B
            )
        );

        ESP_ERROR_CHECK(
            mcpwm_set_duty(
                _unit,
                _timer,
                MCPWM_OPR_A,
                duty
            )
        );

        /*
         * Calling set_signal_low() overrides the normal
         * generator behavior. Restore PWM operation for A.
         */
        ESP_ERROR_CHECK(
            mcpwm_set_duty_type(
                _unit,
                _timer,
                MCPWM_OPR_A,
                MCPWM_DUTY_MODE_0
            )
        );
    }
    else
    {
        /*
         * Reverse:
         * A = LOW
         * B = PWM
         */
        ESP_ERROR_CHECK(
            mcpwm_set_signal_low(
                _unit,
                _timer,
                MCPWM_OPR_A
            )
        );

        ESP_ERROR_CHECK(
            mcpwm_set_duty(
                _unit,
                _timer,
                MCPWM_OPR_B,
                duty
            )
        );

        ESP_ERROR_CHECK(
            mcpwm_set_duty_type(
                _unit,
                _timer,
                MCPWM_OPR_B,
                MCPWM_DUTY_MODE_0
            )
        );
    }

    _lastSpeed = speed;
}

void MotorDriver::stop()
{
    /*
     * Both H-bridge inputs LOW.
     * This assumes LOW/LOW is your bridge's safe stop state.
     */
    ESP_ERROR_CHECK(
        mcpwm_set_signal_low(
            _unit,
            _timer,
            MCPWM_OPR_A
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_set_signal_low(
            _unit,
            _timer,
            MCPWM_OPR_B
        )
    );

    _lastSpeed = 0;
}