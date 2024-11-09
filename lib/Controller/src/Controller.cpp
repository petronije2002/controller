// Controller.cpp
#include "Controller.h"
#include "AS5048my.h"
#include "math.h"

Controller::Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen)
    : _encoder(encoder), _driver(driver), _algorithm(algorithm), _profileGen(profileGen), controlValue(0) {}

void Controller::update(float desiredAngle) {
    // Read the current angle from the encoder
    float currentAngle = _encoder.getAngle();

    // Generate angles vector from the profile generator
    std::vector<float> angles = _profileGen.generateAngles();

    // Get the interpolated position and velocity from the profile generator
    float positionValue = _profileGen.getInterpolatedValue(currentAngle, _profileGen.getPositionProfile(), angles);
    float velocityValue = _profileGen.getInterpolatedValue(currentAngle, _profileGen.getVelocityProfile(), angles);

    // Use positionValue or velocityValue in your PID calculation
    controlValue = _algorithm->calculate(positionValue, currentAngle);

    controlValue = constrain(controlValue, 0.0f, 1.0f);

    // Calculate the duty cycles for the three phases based on the current angle and controlValue
    float theta = currentAngle * (M_PI / 180.0); // Convert to radians

    // Combine controlValue with the sine wave generation for duty cycles
    float dutyA = controlValue * (0.5f * (1 + sin(theta))); // Scale by controlValue
    float dutyB = controlValue * (0.5f * (1 + sin(theta + (2 * M_PI / 3)))); // 120 degrees phase shift
    float dutyC = controlValue * (0.5f * (1 + sin(theta + (4 * M_PI / 3)))); // 240 degrees phase shift

    // Ensure duty cycles are within [0, 1]
    dutyA = constrain(dutyA, 0.0f, 1.0f);
    dutyB = constrain(dutyB, 0.0f, 1.0f);
    dutyC = constrain(dutyC, 0.0f, 1.0f);

    // Set the PWM values for each phase
    _driver.setPWMDutyCycle(dutyA, dutyB, dutyC);
}

void Controller::commandTargetPosition(float targetPosition, float totalDistance) {
    // Generate the S-curve profile based on the desired target position
    _profileGen.generateScurveProfile(totalDistance);
}

float Controller::constrain_(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}
