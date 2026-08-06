#include "core/states/RockMetalCheck.h"

#include <math.h>

#include "actuators/MecanumDrive.h"
#include "core/states/RockApproach.h"

extern MecanumDrive drive;

namespace RockMetalCheck
{

static constexpr uint8_t BASELINE_SAMPLE_COUNT = 20;
static constexpr uint8_t CHECK_SAMPLE_COUNT = 6; //testing? used to be 5...?

static constexpr unsigned long SETTLE_TIME_MS = 2000;
static constexpr float METAL_CHANGE_THRESHOLD_HZ = 150.0f;

enum class Phase
{
    IDLE,
    SETTLING,
    SAMPLING,
    FINISHED,
    FAILED
};

static Phase currentPhase = Phase::IDLE;

static float baselineSum0 = 0.0f;
static float baselineSum1 = 0.0f;

static float baselineAverage0 = 0.0f;
static float baselineAverage1 = 0.0f;

static uint8_t baselineSampleCount = 0;
static bool baselinesReady = false;

static uint8_t currentRockIndex = 0;
static unsigned long phaseStartTime = 0;

static float checkSum = 0.0f;
static uint8_t checkSampleCount = 0;

static float latestCheckValue = 0.0f;
static float latestBaselineValue = 0.0f;
static float latestChange = 0.0f;

static uint8_t latestRockIndex = 0;
static uint8_t latestCoil = 0;

static bool latestMetalFound = false;

static float getDetectorReading(
    const StateMachine::Inputs& inputs,
    uint8_t rockIndex
)
{
    const RockApproach::RockPos& rock =
        RockApproach::ROCK_POSITIONS[rockIndex];

    return rock.coil == 0
        ? inputs.metalMagnitude1
        : inputs.metalMagnitude0;
}

static float getDetectorBaseline(uint8_t rockIndex)
{
    const RockApproach::RockPos& rock =
        RockApproach::ROCK_POSITIONS[rockIndex];

    return rock.coil == 0
        ? baselineAverage1
        : baselineAverage0;
}

void begin()
{
    currentPhase = Phase::IDLE;
}

void resetBaselines()
{
    baselineSum0 = 0.0f;
    baselineSum1 = 0.0f;

    baselineAverage0 = 0.0f;
    baselineAverage1 = 0.0f;

    baselineSampleCount = 0;
    baselinesReady = false;

    latestCheckValue = 0.0f;
    latestBaselineValue = 0.0f;
    latestChange = 0.0f;

    latestRockIndex = 0;
    latestCoil = 0;
    latestMetalFound = false;

    currentPhase = Phase::IDLE;
}

void updateBaselines(const StateMachine::Inputs& inputs)
{
    if (baselinesReady)
    {
        return;
    }

    baselineSum0 += inputs.metalMagnitude0;
    baselineSum1 += inputs.metalMagnitude1;
    ++baselineSampleCount;

    Serial.printf(
        "Metal baseline %u/%u: detector0=%.2f detector1=%.2f\n",
        baselineSampleCount,
        BASELINE_SAMPLE_COUNT,
        inputs.metalMagnitude0,
        inputs.metalMagnitude1
    );

    if (baselineSampleCount < BASELINE_SAMPLE_COUNT)
    {
        return;
    }

    baselineAverage0 =
        baselineSum0 /
        static_cast<float>(BASELINE_SAMPLE_COUNT);

    baselineAverage1 =
        baselineSum1 /
        static_cast<float>(BASELINE_SAMPLE_COUNT);

    baselinesReady = true;

    Serial.printf(
        "Metal baselines ready: detector0=%.2f detector1=%.2f\n",
        baselineAverage0,
        baselineAverage1
    );
}

bool areBaselinesReady()
{
    return baselinesReady;
}

void start(uint8_t rockIndex)
{
    if (!baselinesReady)
    {
        currentPhase = Phase::FAILED;
        return;
    }

    currentRockIndex = rockIndex;

    checkSum = 0.0f;
    checkSampleCount = 0;

    latestMetalFound = false;

    drive.stop();

    phaseStartTime = millis();
    currentPhase = Phase::SETTLING;
}

void update(const StateMachine::Inputs& inputs)
{
    switch (currentPhase)
    {
        case Phase::SETTLING:
        {
            drive.stop();

            if (millis() - phaseStartTime >= SETTLE_TIME_MS)
            {
                currentPhase = Phase::SAMPLING;
            }

            break;
        }

        case Phase::SAMPLING:
        {
            const float reading =
                getDetectorReading(inputs, currentRockIndex);

            checkSum += reading;
            ++checkSampleCount;

            Serial.printf(
                "Metal sample %u/%u: %.2f Hz\n",
                checkSampleCount,
                CHECK_SAMPLE_COUNT,
                reading
            );

            if (checkSampleCount < CHECK_SAMPLE_COUNT)
            {
                break;
            }

            const RockApproach::RockPos& rock =
                RockApproach::ROCK_POSITIONS[currentRockIndex];

            latestCheckValue =
                checkSum /
                static_cast<float>(CHECK_SAMPLE_COUNT);

            latestBaselineValue =
                getDetectorBaseline(currentRockIndex);

            latestChange =
                fabsf(latestCheckValue - latestBaselineValue);

            latestMetalFound =
                latestChange >= METAL_CHANGE_THRESHOLD_HZ;

            latestRockIndex = currentRockIndex;
            latestCoil = rock.coil;

            Serial.printf(
                "Rock %u coil %u: baseline=%.2f check=%.2f "
                "change=%.2f metal=%d\n",
                latestRockIndex,
                latestCoil,
                latestBaselineValue,
                latestCheckValue,
                latestChange,
                latestMetalFound
            );

            currentPhase = Phase::FINISHED;
            break;
        }

        default:
            break;
    }
}

void stop()
{
    drive.stop();
    currentPhase = Phase::IDLE;
}

bool isFinished()
{
    return currentPhase == Phase::FINISHED;
}

bool hasFailed()
{
    return currentPhase == Phase::FAILED;
}

bool metalFound()
{
    return latestMetalFound;
}

float getBaseline0()
{
    return baselineAverage0;
}

float getBaseline1()
{
    return baselineAverage1;
}

float getLatestCheckValue()
{
    return latestCheckValue;
}

float getLatestBaselineValue()
{
    return latestBaselineValue;
}

float getLatestChange()
{
    return latestChange;
}

uint8_t getLatestRockIndex()
{
    return latestRockIndex;
}

uint8_t getLatestCoil()
{
    return latestCoil;
}

} // namespace RockMetalCheck