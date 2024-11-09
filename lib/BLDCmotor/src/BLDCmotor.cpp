#include "BLDCmotor.h"

// Constructor with additional parameters
Motor::Motor(int poles, float kvRating, float maxCurrent, float maxVoltage)
  : polePairs(poles), Kv(kvRating), maxCurrent(maxCurrent), maxVoltage(maxVoltage) {
    currentLimit = maxCurrent;  // Set initial current limit to maximum
    voltageLimit = maxVoltage;   // Set initial voltage limit to maximum
    resistance = 0.0f;           // Initialize resistance (set later)
    inductance = 0.0f;           // Initialize inductance (set later)
    voltage = 0.0f;              // Initialize voltage
    current = 0.0f;              // Initialize current
    angularVelocity = 0.0f;      // Initialize angular velocity
}

// Set winding parameters
void Motor::setWindingParameters(float res, float ind) {
    resistance = res;
    inductance = ind;
}

// Set current
void Motor::setCurrent(float motorCurrent) {
    if (motorCurrent > currentLimit) {
        current = currentLimit; // Limit to max current
    } else {
        current = motorCurrent;
    }
}

// Set voltage
void Motor::setVoltage(float motorVoltage) {
    if (motorVoltage > voltageLimit) {
        voltage = voltageLimit; // Limit to max voltage
    } else {
        voltage = motorVoltage;
    }
}

// Set angular velocity (electrical)
void Motor::setAngularVelocity(float omega) {
    angularVelocity = omega;
}

// Calculate flux linkage: λ = (V - I * R) / ω
float Motor::calculateFluxLinkage() {
    if (angularVelocity != 0) {
        return (voltage - (current * resistance)) / angularVelocity;
    } else {
        return 0;
    }
}

// Calculate back EMF using Kv: E_b = ω / Kv
float Motor::calculateBackEMF() {
    return angularVelocity / Kv;
}

// Calculate torque: T = (3/2) * (P/2) * λ * I
float Motor::calculateTorque() {
    float fluxLinkage = calculateFluxLinkage();
    return (1.5f * polePairs / 2.0f) * fluxLinkage * current;
}

// Get Kv
float Motor::getKv() {
    return Kv;
}

// Additional getters for max current and max voltage
float Motor::getMaxCurrent() {
    return maxCurrent;
}

float Motor::getMaxVoltage() {
    return maxVoltage;
}

// Set current limit
void Motor::setCurrentLimit(float limit) {
    currentLimit = limit;
}

// Set voltage limit
void Motor::setVoltageLimit(float limit) {
    voltageLimit = limit;
}
