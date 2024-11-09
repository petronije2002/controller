


#ifndef PROFILE_GENERATOR_H
#define PROFILE_GENERATOR_H

#include <vector>

class ProfileGenerator {
public:
    ProfileGenerator(float maxVelocity, float maxAcceleration, int numSegments);

    void generateScurveProfile(float totalDistance);
    const std::vector<float>& getPositionProfile() const;
    const std::vector<float>& getVelocityProfile() const;

    float getInterpolatedValue(float currentAngle, const std::vector<float>& profile, const std::vector<float>& angles) const;
    float getPositionForAngle(float currentAngle) const;
    float getVelocityForAngle(float currentAngle) const;

private:
    std::vector<float> positionProfile;
    std::vector<float> velocityProfile;
    std::vector<float> angles; // Define the angles vector here
    float maxVelocity;
    float maxAcceleration;
    int numSegments;
    float totalDistance;

    float sigmoid(float x, float c, float c2) const;
    float calculateCommandedVelocity(float t, float commanded_velocity, float acceleration_time, float constVelocity_time, float c, float c2) const;
    void generateAngleAndVelocityValues(float angleToGo, float commanded_velocity, float acceleration_time, float c, float c2);
    float calculateSegmentPosition(int segment, float t) const;
    float calculateSegmentVelocity(int segment, float t) const;
};

#endif // PROFILE_GENERATOR_H


// class ProfileGenerator {
// public:
//     ProfileGenerator(float maxVelocity, float maxAcceleration, int numSegments);

//     void generateScurveProfile(float totalDistance);
//     const std::vector<float>& getPositionProfile() const;
//     const std::vector<float>& getVelocityProfile() const;

//     float getInterpolatedValue(float currentAngle, const std::vector<float>& profile, const std::vector<float>& angles) const;
//     float getPositionForAngle(float currentAngle) const;
//     float getVelocityForAngle(float currentAngle) const;
//     std::vector<float> ProfileGenerator::generateAngles() const;

// private:
//     std::vector<float> positionProfile;
//     std::vector<float> velocityProfile;
//     std::vector<float> angles; // Define the angles vector here
//     float maxVelocity;
//     float maxAcceleration;
//     int numSegments;
//     float totalDistance;

//     float calculateSegmentPosition(int segment, float t);
//     float calculateSegmentVelocity(int segment, float t);
// };

// #endif // PROFILE_GENERATOR_H
