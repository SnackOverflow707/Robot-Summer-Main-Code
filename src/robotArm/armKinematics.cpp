#include "ArmKinematics.h"
#include <math.h>
#include <Arduino.h>

bool solveIK(float x, float y, float z, int& baseAngle, int& shoulderAngle, int& elbowAngle) {
    // Base yaw: rotate to face the target in the horizontal plane.
    float baseRad = atan2f(y, x);

    // Reduce to the vertical plane containing the base axis and the target:
    // r = horizontal distance from the base axis, h = height above the shoulder joint.
    float r = sqrtf(x * x + y * y);
    float h = z - IK_BASE_HEIGHT;

    float reach = sqrtf(r * r + h * h);
    float maxReach = IK_UPPER_ARM_LENGTH + IK_FOREARM_LENGTH;
    float minReach = fabsf(IK_UPPER_ARM_LENGTH - IK_FOREARM_LENGTH);
    if (reach > maxReach || reach < minReach) {
        return false; // target unreachable
    }

    // Law of cosines for the elbow's interior angle (0 = folded, PI = fully extended).
    float cosElbow = (IK_UPPER_ARM_LENGTH * IK_UPPER_ARM_LENGTH +
                       IK_FOREARM_LENGTH * IK_FOREARM_LENGTH -
                       reach * reach) /
                      (2.0f * IK_UPPER_ARM_LENGTH * IK_FOREARM_LENGTH);
    cosElbow = constrain(cosElbow, -1.0f, 1.0f);
    float elbowRad = acosf(cosElbow);

    // Shoulder angle = angle to target from horizontal, plus the angle
    // between the upper arm and the line to the target (elbow-up solution).
    float cosShoulderOffset = (IK_UPPER_ARM_LENGTH * IK_UPPER_ARM_LENGTH +
                                reach * reach -
                                IK_FOREARM_LENGTH * IK_FOREARM_LENGTH) /
                               (2.0f * IK_UPPER_ARM_LENGTH * reach);
    cosShoulderOffset = constrain(cosShoulderOffset, -1.0f, 1.0f);
    float shoulderRad = atan2f(h, r) + acosf(cosShoulderOffset);

    float baseDeg = baseRad * 180.0f / PI;
    float shoulderDeg = shoulderRad * 180.0f / PI;
    float elbowDeg = elbowRad * 180.0f / PI;

    baseAngle     = (int)roundf(IK_BASE_ZERO_DEG     + IK_BASE_SIGN     * baseDeg);
    shoulderAngle = (int)roundf(IK_SHOULDER_ZERO_DEG + IK_SHOULDER_SIGN * shoulderDeg);
    elbowAngle    = (int)roundf(IK_ELBOW_ZERO_DEG    + IK_ELBOW_SIGN    * elbowDeg);

    return true;
}
