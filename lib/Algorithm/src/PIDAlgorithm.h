// PIDController.h
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Algorithm.h" // Include your base Algorithm header

class PIDController : public Algorithm {
public:
    PIDController(float _kp, float _ki, float _kd);
    void init() override; // Override the init method
    float calculate(float desired, float current); // Calculate method

private:
    float kp, ki, kd;
    float integral, previous_error;
};

#endif // PIDCONTROLLER_H
