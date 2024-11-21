// PIDController.cpp
#include "PIDAlgorithm.h"

PIDController::PIDController(float _kp, float _ki, float _kd) 
    : kp(_kp), ki(_ki), kd(_kd), integral(0), previous_error(0) {}




void PIDController::init() {
    integral = 0;          // Reset the integral term
    previous_error = 0;    // Reset the previous error
    // std::cout << "PIDController initialized" << std::endl;
}



float PIDController::calculate(float desired, float current) {
    float error = desired - current; // Calculate error
    integral += error; // Update integral
    float derivative = error - previous_error; // Calculate derivative
    previous_error = error; // Update previous error

    // Calculate control output
    float output = kp * error;
    // + ki * integral + kd * derivative;
    return output; // Return control output
}
