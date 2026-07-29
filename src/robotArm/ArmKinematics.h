#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

// Inverse kinematics for the base/shoulder/elbow chain.
// Solves for the "elbow up" configuration only.
//
// Frame: x/y form the horizontal plane, z is vertical, origin at the
// base rotation axis at the shoulder's height reference (see IK_BASE_HEIGHT).

// ---- Arm geometry (mm) — REPLACE WITH MEASURED VALUES ----
#define IK_UPPER_ARM_LENGTH   200.0f   // shoulder joint to elbow joint
#define IK_FOREARM_LENGTH     200.0f   // elbow joint to wrist joint
#define IK_BASE_HEIGHT          80.0f  // base rotation axis to shoulder joint, along z

// ---- Servo calibration — REPLACE WITH MEASURED VALUES ----
// Maps the solver's math-convention joint angles (0 deg = arm pointing
// along +x/horizontal, increasing counterclockwise) to servo command
// angles: servoAngle = zeroDeg + sign * mathAngleDeg.
#define IK_BASE_ZERO_DEG        0.0f
#define IK_BASE_SIGN            1
#define IK_SHOULDER_ZERO_DEG    0.0f
#define IK_SHOULDER_SIGN        1
#define IK_ELBOW_ZERO_DEG       0.0f
#define IK_ELBOW_SIGN           1

// Solves for the servo angles that place the wrist joint at (x, y, z),
// given in the same units as the length constants above.
// Returns false if the target is outside the arm's reach and leaves the
// output angles unset.
bool solveIK(float x, float y, float z, int& baseAngle, int& shoulderAngle, int& elbowAngle);

#endif // ARM_KINEMATICS_H
