#include "comms/UART.h"
#include "config/pins.h"

#include <cstring>

namespace UART
{
static constexpr uint8_t MAX_PAYLOAD = 32;  // Add this before payload[]
static constexpr int UART_NUMBER = 1;
static constexpr int UART_RX_PIN = 16;
static constexpr int UART_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;
static uint8_t payload[MAX_PAYLOAD];

static constexpr uint8_t FRAME_SYNC = 0xAA;

static HardwareSerial uart(UART_NUMBER);

enum FrameState
{
    WAIT_SYNC,
    WAIT_LENGTH,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
};

static FrameState state = WAIT_SYNC;
static uint8_t payloadLength = 0;
static uint8_t payloadIndex = 0;

static Data latestData =
{
    .mag1 = 0,
    .mag2 = 0,
    .mask = 0,
    .frameCount = 0,
    .lastUpdateMs = 0,
    .valid = false
};

static void resetParser()
{
    state = WAIT_SYNC;
    payloadLength = 0;
    payloadIndex = 0;
}

static void processByte(uint8_t byte)
{
    switch (state)
    {
        case WAIT_SYNC:
        {
            if (byte == FRAME_SYNC)
            {
                state = WAIT_LENGTH;
            }

            break;
        }

        case WAIT_LENGTH:
        {
            payloadLength = byte;
            payloadIndex = 0;

            if (payloadLength > 0 &&
                payloadLength <= MAX_PAYLOAD)
            {
                state = WAIT_PAYLOAD;
            }
            else
            {
                resetParser();
            }

            break;
        }

        case WAIT_PAYLOAD:
        {
            payload[payloadIndex++] = byte;

            if (payloadIndex >= payloadLength)
            {
                state = WAIT_CHECKSUM;
            }

            break;
        }

        case WAIT_CHECKSUM:
        {
            uint8_t checksum = 0;

            for (uint8_t i = 0; i < payloadLength; i++)
            {
                checksum ^= payload[i];
            }

            bool validChecksum = (checksum == byte);

            if (validChecksum && payloadLength >= 5)
            {
                memcpy(&latestData.mag1, &payload[0], 2);
                memcpy(&latestData.mag2, &payload[2], 2);

                latestData.mask = payload[4];
                latestData.frameCount++;
                latestData.lastUpdateMs = millis();
                latestData.valid = true;
            }

            resetParser();
            break;
        }
    }
}

void begin()
{
    uart.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    // Internal pull-up:
    // switch open       -> HIGH -> mag2
    // switch connected to GND -> LOW -> mag1
    pinMode(FREQ_SWITCH_PIN, INPUT_PULLUP);

    resetParser();
}

void update()
{
    constexpr int MAX_BYTES_PER_UPDATE = 32;

    int bytesProcessed = 0;

    while (uart.available() > 0 &&
           bytesProcessed < MAX_BYTES_PER_UPDATE)
    {
        int received = uart.read();

        if (received < 0)
        {
            return;
        }

        processByte(static_cast<uint8_t>(received));

        bytesProcessed++;
    }
}

Data getData()
{
    return latestData;
}

bool isMag1Selected()
{
    return digitalRead(FREQ_SWITCH_PIN) == LOW;
}

bool isMag2Selected()
{
    return !isMag1Selected();
}

uint16_t getSelectedMagnitude()
{
    if (isMag1Selected())
    {
        return latestData.mag1;
    }

    return latestData.mag2;
}

uint8_t getSelectedFrequency()
{
    if (isMag1Selected())
    {
        return 1;
    }

    return 2;
}

bool isSelectedDetected()
{
    if (isMag1Selected())
    {
        return (latestData.mask & 0x01) != 0;
    }

    return (latestData.mask & 0x02) != 0;
}

} // namespace UART