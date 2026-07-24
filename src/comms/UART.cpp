#include "comms/UART.h"
#include "config/pins.h"

#include <cstring>

namespace UART
{

static constexpr uint8_t MAX_PAYLOAD = 32;
static constexpr uint8_t METAL_DETECTOR_COUNT = 2;

static constexpr int UART_NUMBER = 1;
static constexpr int UART_RX_PIN = 16;
static constexpr int UART_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;

static constexpr uint8_t FRAME_SYNC = 0xAA;

static constexpr uint8_t FRAME_TYPE_IR = 0x01;
static constexpr uint8_t FRAME_TYPE_METAL = 0x02;

static HardwareSerial uart(UART_NUMBER);

static uint8_t payload[MAX_PAYLOAD];

enum FrameState
{
    WAIT_SYNC,
    WAIT_TYPE,
    WAIT_LENGTH,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
};

static FrameState state = WAIT_SYNC;

static uint8_t frameType = 0;
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

// One entry for detector 0 and one for detector 1.
static MetalData latestMetalData[METAL_DETECTOR_COUNT] =
{
    {
        .frequencyHz = 0.0f,
        .frameCount = 0,
        .lastUpdateMs = 0,
        .valid = false
    },
    {
        .frequencyHz = 0.0f,
        .frameCount = 0,
        .lastUpdateMs = 0,
        .valid = false
    }
};

// --------------------------------------------------
// Parser helpers
// --------------------------------------------------

static void resetParser()
{
    state = WAIT_SYNC;

    frameType = 0;
    payloadLength = 0;
    payloadIndex = 0;
}

static uint8_t calculateChecksum()
{
    uint8_t checksum = 0;

    checksum ^= frameType;
    checksum ^= payloadLength;

    for (uint8_t i = 0; i < payloadLength; ++i)
    {
        checksum ^= payload[i];
    }

    return checksum;
}

static void processIRFrame()
{
    if (payloadLength != 5)
    {
        return;
    }

    latestData.mag1 =
        static_cast<uint16_t>(payload[0]) |
        (static_cast<uint16_t>(payload[1]) << 8);

    latestData.mag2 =
        static_cast<uint16_t>(payload[2]) |
        (static_cast<uint16_t>(payload[3]) << 8);

    latestData.mask = payload[4];

    latestData.frameCount++;
    latestData.lastUpdateMs = millis();
    latestData.valid = true;
}

static void processMetalFrame()
{
    // Payload:
    // byte 0: detector ID
    // bytes 1-4: frequency as a float
    static constexpr uint8_t EXPECTED_LENGTH =
        1 + sizeof(float);

    if (payloadLength != EXPECTED_LENGTH)
    {
        return;
    }

    const uint8_t detectorId = payload[0];

    if (detectorId >= METAL_DETECTOR_COUNT)
    {
        return;
    }

    float frequencyHz = 0.0f;

    memcpy(
        &frequencyHz,
        &payload[1],
        sizeof(float)
    );

    latestMetalData[detectorId].frequencyHz =
        frequencyHz;

    latestMetalData[detectorId].frameCount++;

    latestMetalData[detectorId].lastUpdateMs =
        millis();

    latestMetalData[detectorId].valid = true;
}

static void processCompleteFrame()
{
    switch (frameType)
    {
        case FRAME_TYPE_IR:
            processIRFrame();
            break;

        case FRAME_TYPE_METAL:
            processMetalFrame();
            break;

        default:
            break;
    }
}

static void processByte(uint8_t byte)
{
    switch (state)
    {
        case WAIT_SYNC:
        {
            if (byte == FRAME_SYNC)
            {
                state = WAIT_TYPE;
            }

            break;
        }

        case WAIT_TYPE:
        {
            frameType = byte;
            state = WAIT_LENGTH;
            break;
        }

        case WAIT_LENGTH:
        {
            payloadLength = byte;
            payloadIndex = 0;

            if (
                payloadLength > 0 &&
                payloadLength <= MAX_PAYLOAD
            )
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
            const uint8_t expectedChecksum =
                calculateChecksum();

            if (expectedChecksum == byte)
            {
                processCompleteFrame();
            }

            resetParser();
            break;
        }
    }
}

// --------------------------------------------------
// Public functions
// --------------------------------------------------

void begin()
{
    uart.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    resetParser();
}

void update()
{
    constexpr int MAX_BYTES_PER_UPDATE = 64;

    int bytesProcessed = 0;

    while (
        uart.available() > 0 &&
        bytesProcessed < MAX_BYTES_PER_UPDATE
    )
    {
        const int received = uart.read();

        if (received < 0)
        {
            return;
        }

        processByte(
            static_cast<uint8_t>(received)
        );

        ++bytesProcessed;
    }
}

Data getData()
{
    return latestData;
}

MetalData getMetalData(uint8_t detectorId)
{
    if (detectorId >= METAL_DETECTOR_COUNT)
    {
        return MetalData
        {
            .frequencyHz = 0.0f,
            .frameCount = 0,
            .lastUpdateMs = 0,
            .valid = false
        };
    }

    return latestMetalData[detectorId];
}

bool isMag1Selected()
{
    return (latestData.mask & 0x04) != 0;
}

bool isMag2Selected()
{
    return !isMag1Selected();
}

uint16_t getSelectedMagnitude()
{
    return isMag1Selected()
        ? latestData.mag1
        : latestData.mag2;
}

uint8_t getSelectedFrequency()
{
    return isMag1Selected() ? 1 : 2;
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