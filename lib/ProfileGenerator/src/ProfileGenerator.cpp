#include "ProfileGenerator.h"
#include <cmath>  // For exp()

#include "ProfileGenerator.h"
#include <cmath>
#include <iostream>



// Constructor
ProfileGenerator::ProfileGenerator(float maxVelocity, float maxAcceleration, int numSegments)
    : maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), numSegments(numSegments), totalDistance(0.0f) {}

// Sigmoid function to create an S-curve
float ProfileGenerator::sigmoid(float x, float c, float c2) const {
    return 1 / (1 + exp(-c * (x - c2)));
}

// Calculate the commanded velocity for the current time
float ProfileGenerator::calculateCommandedVelocity(float t, float commanded_velocity, float acceleration_time, float constVelocity_time, float c, float c2) const {
    float v_cmd = commanded_velocity;

    if (t <= acceleration_time) {
        v_cmd = commanded_velocity * sigmoid(t, c, c2);
    }
    if (t > acceleration_time && t <= constVelocity_time + acceleration_time) {
        v_cmd = commanded_velocity;
    }
    if (t > (constVelocity_time + acceleration_time)) {
        v_cmd = commanded_velocity - commanded_velocity * sigmoid(t - acceleration_time - constVelocity_time, c, c2);
    }

    return v_cmd;
}

// Generate the angle and velocity values
void ProfileGenerator::generateAngleAndVelocityValues(float angleToGo, float commanded_velocity, float acceleration_time, float c, float c2) {
    // Compute total time based on commanded velocity
    float total_time = angleToGo / commanded_velocity;

    // Calculate time spent at constant velocity
    float constVelocity_time = total_time - 2 * acceleration_time;

    if (constVelocity_time <= 0) {
        // If no constant velocity phase, recalculate commanded velocity to fit the acceleration phase
        commanded_velocity = 2 * (acceleration_time) / angleToGo;
        total_time = angleToGo / commanded_velocity;
        constVelocity_time = 0;
    }

    // Initialize vectors to store angle and velocity values
    positionProfile.clear();
    velocityProfile.clear();
    angles.clear();

    // Calculate step size for each phase
    int num_accel_segments = 25;  // Example value for acceleration segments
    int num_const_segments = 0;   // Example value for constant velocity segments
    float step_size_ad = acceleration_time / (float)(num_accel_segments);
    float step_size_const = constVelocity_time / (float)(num_const_segments);

    // Acceleration Phase
    float current_time = 0;
    for (int i = 0; i <= num_accel_segments; ++i) {
        current_time = i * step_size_ad;
        float angle = positionProfile.empty() ? 0 : positionProfile.back() + current_time * calculateCommandedVelocity(current_time, commanded_velocity, acceleration_time, constVelocity_time, c, c2);
        positionProfile.push_back(angle);
        velocityProfile.push_back(calculateCommandedVelocity(current_time, commanded_velocity, acceleration_time, constVelocity_time, c, c2));
    }

    // Constant Velocity Phase
    for (int i = num_accel_segments; i < num_accel_segments + num_const_segments; ++i) {
        current_time = acceleration_time + (i - num_accel_segments) * step_size_const;
        positionProfile.push_back(positionProfile.back() + commanded_velocity * step_size_const);
        velocityProfile.push_back(commanded_velocity);
    }

    // Deceleration Phase (reverse S-curve)
    for (int i = num_accel_segments + num_const_segments; i < num_accel_segments + num_const_segments + num_accel_segments; ++i) {
        current_time = acceleration_time + constVelocity_time + (i - num_accel_segments - num_const_segments) * step_size_ad;
        float reverse_factor = num_accel_segments + num_const_segments + num_accel_segments - i;
        positionProfile.push_back(positionProfile.back() + reverse_factor * calculateCommandedVelocity(current_time, commanded_velocity, acceleration_time, constVelocity_time, c, c2));
        velocityProfile.push_back(commanded_velocity - calculateCommandedVelocity(current_time, commanded_velocity, acceleration_time, constVelocity_time, c, c2));
    }
}

// Generate the S-curve profile for the given distance
void ProfileGenerator::generateScurveProfile(float totalDistance) {
    this->totalDistance = totalDistance;

    // Set up the profile generation parameters
    float acceleration_time = 0.3f;  // Example acceleration time
    float c = 5.0f;  // Sigmoid parameter for shaping the curve
    float c2 = 0.5f; // Sigmoid center (midpoint of transition)
    float commanded_velocity = maxVelocity;  // Example commanded velocity

    // Call the function to generate the angle and velocity values
    generateAngleAndVelocityValues(totalDistance, commanded_velocity, acceleration_time, c, c2);
}

// Get the position profile
const std::vector<float>& ProfileGenerator::getPositionProfile() const {
    return positionProfile;
}

// Get the velocity profile
const std::vector<float>& ProfileGenerator::getVelocityProfile() const {
    return velocityProfile;
}

// Interpolate a value (used for getting position/velocity at a given angle)
float ProfileGenerator::getInterpolatedValue(float currentAngle, const std::vector<float>& profile, const std::vector<float>& angles) const {
    for (size_t i = 0; i < angles.size() - 1; ++i) {
        if (currentAngle >= angles[i] && currentAngle < angles[i + 1]) {
            float t = (currentAngle - angles[i]) / (angles[i + 1] - angles[i]);
            return profile[i] + t * (profile[i + 1] - profile[i]);
        }
    }
    return 0.0f;  // Default return if out of bounds
}

// Get the position for a given angle
float ProfileGenerator::getPositionForAngle(float currentAngle) const {
    return getInterpolatedValue(currentAngle, positionProfile, angles);
}

// Get the velocity for a given angle
float ProfileGenerator::getVelocityForAngle(float currentAngle) const {
    return getInterpolatedValue(currentAngle, velocityProfile, angles);
}





// ProfileGenerator::ProfileGenerator(float maxVelocity, float maxAcceleration, int numSegments)
//     : maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), numSegments(numSegments) {
//     positionProfile.resize(numSegments);
//     velocityProfile.resize(numSegments);
// }

// // void ProfileGenerator::generateScurveProfile(float totalDistance) {
// //     this->totalDistance = totalDistance;
    
// //     float t = 0.0;
// //     float segmentTime = 1.0f / (float)(numSegments - 1);  // Normalized time between 0 and 1

// //     for (int i = 0; i < numSegments; ++i) {
// //         t = i * segmentTime;
        
// //         // Calculate the S-curve position and velocity for each time step 't'
// //         positionProfile[i] = calculateSegmentPosition(i, t);
// //         velocityProfile[i] = calculateSegmentVelocity(i, t);
// //     }
// // }

// float ProfileGenerator::calculateSegmentPosition(int segment, float t) {
//     // S-curve using sigmoid function
//     float sigmoid_t = 1.0 / (1.0 + exp(-10.0 * (t - 0.5)));  // Sigmoid shape for position
//     return totalDistance * sigmoid_t;  // Scale sigmoid by total distance
// }

// float ProfileGenerator::calculateSegmentVelocity(int segment, float t) {
//     // Derivative of the sigmoid function for velocity
//     float sigmoid_t = exp(-10.0 * (t - 0.5)) / pow(1.0 + exp(-10.0 * (t - 0.5)), 2);
//     return maxVelocity * sigmoid_t;  // Scale by max velocity
// }

// const std::vector<float>& ProfileGenerator::getPositionProfile() const {
//     return positionProfile;
// }

// const std::vector<float>& ProfileGenerator::getVelocityProfile() const {
//     return velocityProfile;
// }

// // Assuming you want linear interpolation between the values in the profile.
// void ProfileGenerator::generateScurveProfile(float totalDistance) {
//     this->totalDistance = totalDistance;
    
//     float t = 0.0;
//     float segmentTime = 1.0f / (float)(numSegments - 1);  // Normalized time between 0 and 1

//     angles.resize(numSegments); // Resize the angles vector

//     // Assuming totalDistance is the total angle the motor needs to cover
//     float totalAngle = totalDistance; // You can rename or adjust this based on your use case

//     for (int i = 0; i < numSegments; ++i) {
//         t = i * segmentTime;
        
//         // Calculate the S-curve position and velocity for each time step 't'
//         positionProfile[i] = calculateSegmentPosition(i, t);
//         velocityProfile[i] = calculateSegmentVelocity(i, t);

//         // Linearly distribute the angles between 0 and totalAngle
//         angles[i] = totalAngle * t;
//     }
// }

// std::vector<float> ProfileGenerator::generateAngles() const {
//     std::vector<float> angles(numSegments);
//     float segmentAngle = totalDistance / (numSegments - 1); // Adjust this based on your total distance and segments

//     for (int i = 0; i < numSegments; ++i) {
//         angles[i] = i * segmentAngle;
//     }

//     return angles;
// }

