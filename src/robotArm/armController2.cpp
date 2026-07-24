#include "ArmController2.h"


ArmController2::ArmController2()
    : _baseServo(PIN_BASE, BASE_SERVO),
      _shoulderAServo(PIN_SHOULDER_A, SHOULDERA_SERVO),
      _shoulderBServo(PIN_SHOULDER_B, SHOULDERB_SERVO),
      _elbowServo(PIN_ELBOW, ELBOW_SERVO),
      _wristServo(PIN_WRIST, WRIST_SERVO),
      _clawServo(PIN_CLAW, CLAW_SERVO), 
      clawSwitch(PIN_SWITCH) 
{
    //initialize the servo array here and point it at the actual servo objects 
    servos[BASE_SERVO] = &_baseServo;
    servos[SHOULDERA_SERVO] = &_shoulderAServo;
    servos[SHOULDERB_SERVO] = &_shoulderBServo;
    servos[ELBOW_SERVO] = &_elbowServo;
    servos[WRIST_SERVO] = &_wristServo;
    servos[CLAW_SERVO] = &_clawServo;
}

void ArmController2::begin() 
{

    ESP32PWM::allocateTimer(0); 
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (int i = 0; i < NSERVOS; i++) {
        servos[i]->attach();
    }

    configureLimits(); 
    goHome();
    clawSwitch.begin(); 
    
}


void ArmController2::configureLimits() {

    for (int i = 0; i < NSERVOS; i++) {
        if (i == CLAW_SERVO) {
            servos[i]->setLimits(MIN_ANGLE, MAX_ANGLE_CLAW);
        } 
        else {
            servos[i]->setLimits(MIN_ANGLE, MAX_ANGLE);
        }
    }
}

void ArmController2::moveJoint(ArmServo& servo, int angle, int omega, int offset) {
    angle += offset;
    servo.moveTo(angle, omega);
}

// Steps both servos together, one loop, so they arrive at their targets
// simultaneously instead of one finishing before the other starts.
void ArmController2::moveJointPair(ArmServo& servoA, ArmServo& servoB, int angle, int omega, int offset) {
    int targetA = angle + offset;
    int targetB = (MAX_ANGLE - angle) + offset;

    int startA = servoA.read();
    int startB = servoB.read();

    // No known position yet - jump directly, same as ArmServo::moveTo().
    if (startA < 0) { servoA.write(targetA); startA = targetA; }
    if (startB < 0) { servoB.write(targetB); startB = targetB; }

    int deltaA = targetA - startA;
    int deltaB = targetB - startB;

    int steps = max(abs(deltaA), abs(deltaB));
    if (steps == 0) return;

    // Duration set by the joint with the larger sweep moving at omega deg/s;
    // the other joint is stepped proportionally slower so both finish together.
    float totalTime_ms = ((float)steps / omega) * 1000.0f;
    float stepDelay_ms = totalTime_ms / steps;

    float stepA = (float)deltaA / steps;
    float stepB = (float)deltaB / steps;

    float curA = startA;
    float curB = startB;

    for (int i = 0; i < steps; i++) {
        curA += stepA;
        curB += stepB;
        servoA.write((int)(curA + 0.5f));
        servoB.write((int)(curB + 0.5f));
        delay((uint32_t)stepDelay_ms);
    }
}

void ArmController2::goHome()
{
    for (int i = 0; i < NSERVOS; i++) {
        if (i == CLAW_SERVO) {
            moveClaw(_clawServo, HOME_CLAW, OMEGA_CLAW); 
        }
        else {
            moveJoint(*servos[i], homeArray[i], omegas[i], offsets[i]);
        }
    }
}


void ArmController2::moveClaw(ArmServo& servo, int angle, int omega) {    
    angle += CLAW_OFFSET; 
    servo.moveTo(angle, omega);
}

void ArmController2::openClaw() {
    _clawServo.moveTo(MIN_ANGLE, OMEGA_CLAW); 
}

void ArmController2::closeClaw() {
    _clawServo.moveTo(MAX_ANGLE_CLAW, OMEGA_CLAW); 
}




// Public methods 
void ArmController2::setBase(int angle) {
    moveJoint(_baseServo, angle, OMEGA_BASE, BASE_OFFSET);
}
void ArmController2::setShoulder(int angle) {
    moveJointPair(_shoulderAServo, _shoulderBServo, angle, OMEGA_SHOULDER, SHOULDER_OFFSET);
}
void ArmController2::setElbow(int angle) {
    moveJoint(_elbowServo, angle, OMEGA_ELBOW, ELBOW_OFFSET);
}
void ArmController2::setWrist(int angle) {
    moveJoint(_wristServo, angle, OMEGA_WRIST, WRIST_OFFSET);
}
void ArmController2::setClaw(int angle) {
    moveClaw(_clawServo, angle, OMEGA_CLAW);
}

int ArmController2::getBase() const     { return _baseServo.read() - BASE_OFFSET;}
int ArmController2::getShoulder() const { return _shoulderAServo.read() - SHOULDER_OFFSET; }
int ArmController2::getElbow() const    { return _elbowServo.read() - ELBOW_OFFSET; }
int ArmController2::getWrist() const    { return _wristServo.read() - WRIST_OFFSET; }
int ArmController2::getClaw() const     { return (_clawServo.read() - CLAW_OFFSET); }