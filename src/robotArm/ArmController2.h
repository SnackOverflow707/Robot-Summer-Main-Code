#ifndef ARM_CONTROLLER2_H
#define ARM_CONTROLLER2_H

#include <ArmServo.h> 
#include <ESP32PWM.h>
#include <microswitch.h> 

//general definitions 
#define NSERVOS 6 
#define BASE_SERVO 0
#define SHOULDERA_SERVO 1
#define SHOULDERB_SERVO 2
#define ELBOW_SERVO 3
#define WRIST_SERVO 4
#define CLAW_SERVO  5

// Pin definitions for the servos
#define PIN_BASE   41 
#define PIN_SHOULDER_A 47
#define PIN_SHOULDER_B 38
#define PIN_ELBOW     39 
#define PIN_WRIST    40 
#define PIN_CLAW     21

#define PIN_SWITCH 17

// Home position angles
#define HOME_BASE     90
#define HOME_SHOULDER 160
#define HOME_ELBOW    80
#define HOME_WRIST    0
#define HOME_CLAW      0 // fully open 

//offset angles based on servo mounting, if needed 
#define BASE_OFFSET 0 
#define SHOULDER_OFFSET 5  
#define ELBOW_OFFSET 0
#define WRIST_OFFSET 0
#define CLAW_OFFSET 0

//joint speeds 
#define DEFAULT_OMEGA 60 //in deg/s 
#define OMEGA_BASE 75
#define OMEGA_SHOULDER 30   
#define OMEGA_ELBOW DEFAULT_OMEGA
#define OMEGA_WRIST DEFAULT_OMEGA
#define OMEGA_CLAW 100 //in deg/s  

//specific joint angles. Default min/max are in ArmServo.h  
#define MAX_ANGLE_CLAW 255

class ArmController2
{
public:

    ArmController2();

    MicroSwitch clawSwitch;

    void begin();

    void setBase(int angle);
    void setShoulder(int angle);
    void setElbow(int angle);
    void setWrist(int angle);
    void setClaw(int angle);

    int getBase() const;
    int getShoulder() const;
    int getElbow() const;
    int getWrist() const;
    int getClaw() const;

    void goHome();
    void configureLimits(); 

    void openClaw(); 
    void closeClaw(); 

private:

    ArmServo _baseServo = ArmServo(PIN_BASE, BASE_SERVO);
    ArmServo _shoulderAServo = ArmServo(PIN_SHOULDER_A, SHOULDERA_SERVO);
    ArmServo _shoulderBServo = ArmServo(PIN_SHOULDER_B, SHOULDERB_SERVO);
    ArmServo _elbowServo    = ArmServo(PIN_ELBOW,       ELBOW_SERVO);
    ArmServo _wristServo    = ArmServo(PIN_WRIST,       WRIST_SERVO);
    ArmServo _clawServo = ArmServo(PIN_CLAW, CLAW_SERVO, 1000, 2000, MIN_ANGLE, MAX_ANGLE_CLAW);

    const int homeArray[NSERVOS] = {HOME_BASE, HOME_SHOULDER, MAX_ANGLE - HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, HOME_CLAW};
    const int offsets[NSERVOS] = {BASE_OFFSET, SHOULDER_OFFSET, SHOULDER_OFFSET, ELBOW_OFFSET, WRIST_OFFSET, CLAW_OFFSET};
    const int omegas[NSERVOS] = {OMEGA_BASE, OMEGA_SHOULDER, OMEGA_SHOULDER, OMEGA_ELBOW, OMEGA_WRIST, OMEGA_CLAW}; 
    ArmServo* servos[NSERVOS];  //no initialization because it's dangerous 

    void moveJoint(ArmServo& servo, int angle, int omega, int offset); 
    void moveClaw(ArmServo& servo, int angle, int omega);

};

#endif // ARM_CONTROLLER2_H 