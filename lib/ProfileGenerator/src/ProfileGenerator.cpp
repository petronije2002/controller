#include "ProfileGenerator.h"
#include <cmath>  // For exp()

ProfileGenerator::ProfileGenerator(float maxVelocity, float maxAcceleration, int numSegments)
    : maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), numSegments(numSegments) {
    positionProfile.resize(numSegments);
    velocityProfile.resize(numSegments);
}

// void ProfileGenerator::generateScurveProfile(float totalDistance) {
//     this->totalDistance = totalDistance;
    
//     float t = 0.0;
//     float segmentTime = 1.0f / (float)(numSegments - 1);  // Normalized time between 0 and 1

//     for (int i = 0; i < numSegments; ++i) {
//         t = i * segmentTime;
        
//         // Calculate the S-curve position and velocity for each time step 't'
//         positionProfile[i] = calculateSegmentPosition(i, t);
//         velocityProfile[i] = calculateSegmentVelocity(i, t);
//     }
// }

float ProfileGenerator::calculateSegmentPosition(int segment, float t) {
    // S-curve using sigmoid function
    float sigmoid_t = 1.0 / (1.0 + exp(-10.0 * (t - 0.5)));  // Sigmoid shape for position
    return totalDistance * sigmoid_t;  // Scale sigmoid by total distance
}

float ProfileGenerator::calculateSegmentVelocity(int segment, float t) {
    // Derivative of the sigmoid function for velocity
    float sigmoid_t = exp(-10.0 * (t - 0.5)) / pow(1.0 + exp(-10.0 * (t - 0.5)), 2);
    return maxVelocity * sigmoid_t;  // Scale by max velocity
}

const std::vector<float>& ProfileGenerator::getPositionProfile() const {
    return positionProfile;
}

const std::vector<float>& ProfileGenerator::getVelocityProfile() const {
    return velocityProfile;
}

// Assuming you want linear interpolation between the values in the profile.
void ProfileGenerator::generateScurveProfile(float totalDistance) {
    this->totalDistance = totalDistance;
    
    float t = 0.0;
    float segmentTime = 1.0f / (float)(numSegments - 1);  // Normalized time between 0 and 1

    angles.resize(numSegments); // Resize the angles vector

    // Assuming totalDistance is the total angle the motor needs to cover
    float totalAngle = totalDistance; // You can rename or adjust this based on your use case

    for (int i = 0; i < numSegments; ++i) {
        t = i * segmentTime;
        
        // Calculate the S-curve position and velocity for each time step 't'
        positionProfile[i] = calculateSegmentPosition(i, t);
        velocityProfile[i] = calculateSegmentVelocity(i, t);

        // Linearly distribute the angles between 0 and totalAngle
        angles[i] = totalAngle * t;
    }
}

std::vector<float> ProfileGenerator::generateAngles() const {
    std::vector<float> angles(numSegments);
    float segmentAngle = totalDistance / (numSegments - 1); // Adjust this based on your total distance and segments

    for (int i = 0; i < numSegments; ++i) {
        angles[i] = i * segmentAngle;
    }

    return angles;
}

