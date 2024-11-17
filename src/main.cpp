#include <Arduino.h>
#include "ProfileGenerator.h"

// Initialize the ProfileGenerator
float maxVelocity = 4.0f;   // Maximum velocity
float maxAcceleration = 2.0f;  // Maximum acceleration
int numSegments = 50;  // Number of segments
ProfileGenerator profileGen(maxVelocity, maxAcceleration, numSegments);

// Variables to store the profiles
const std::vector<float>* positionProfile;
const std::vector<float>* velocityProfile;

const std::vector<float>* currentTime;

// Iterator for loop testing
size_t currentIndex = 0;

void setup() {
    // Start serial communication
    Serial.begin(115200);

    // Generate the S-curve profile
    float totalDistance = 3.0f;       // Example distance to travel
    float commandedVelocity = 1.4;   // Example commanded velocity
    
}

void loop() {


    uint64_t before_ = micros();

    profileGen.generateScurveProfile(3, 1.4);
    uint64_t after_ = micros();


    // Get the generated profiles
    positionProfile = &profileGen.getPositionProfile();
    velocityProfile = &profileGen.getVelocityProfile();

    currentTime = &profileGen.getTimeProfile();

    float currentAngle = 0.085;

    float targetVelocity = profileGen.getVelocityForAngle(currentAngle);
    
    // Print the profiles for verification
    // Serial.println("Position Profile:");
    // for (size_t i = 0; i < positionProfile->size(); ++i) {
    //     Serial.printf("%f, %f, %f \n",currentTime->at(i), positionProfile->at(i),velocityProfile->at(i) );
    // }

    Serial.printf("Duration: %f \n", targetVelocity);

    // Serial.println("\nVelocity Profile:");
    // for (size_t i = 0; i < velocityProfile->size(); ++i) {
    //     Serial.printf("Step %d: %f\n", i, velocityProfile->at(i));
    // }


    //  Serial.println("Time Profile:");
    // for (size_t i = 0; i < currentTime->size(); ++i) {
    //     Serial.printf("Step %d: %f\n", i, currentTime->at(i));
    // }

    // Serial.println("\nTesting profile in loop...");


    delay(1000);
    // // Simulate running through the profile step by step
    // if (currentIndex < positionProfile->size()) {
    //     float currentPosition = positionProfile->at(currentIndex);
    //     float currentVelocity = velocityProfile->at(currentIndex);

    //     // Print the current step
    //     Serial.printf("Step %d: Position = %f, Velocity = %f\n", currentIndex, currentPosition, currentVelocity);

    //     // Increment to the next step
    //     currentIndex++;
    // } else {
    //     // All steps completed, reset or stop
    //     Serial.println("Profile completed.");
    //     delay(5000); // Wait for 5 seconds before restarting
    //     currentIndex = 0; // Reset index to start over (optional)
    // }

    // delay(100); // Simulate a time delay between steps (adjust as needed)
}
