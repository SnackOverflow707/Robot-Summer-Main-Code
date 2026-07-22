#pragma once

class MicroSwitch {
public:
    MicroSwitch(int pin);
    void begin();
    bool isPressed() const;
    bool fullyClosed(int durationMs); 
    //choose a duration based on how long you want to check for 

private:
    int _pin;
};