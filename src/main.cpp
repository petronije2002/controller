#include <Arduino.h>
#include "ProfileGenerator.h"

#include "AS5048my.h"
#include "Driver.h"
#include "PIDAlgorithm.h" 

#include "Controller.h"

// AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen

// // Initialize the ProfileGenerator
// float maxVelocity = 4.0f;   // Maximum velocity
// float maxAcceleration = 2.0f;  // Maximum acceleration
// int numSegments = 50;  // Number of segments


ProfileGenerator profileGen(5, 1, 20);

AS5048 encoder_(5);

Driver driver_(13,15,12,14,27,26);

Algorithm* pid = new PIDController(1.0f, 0.1f, 0.05f); // Create a pointer to PIDController

Controller controller_( encoder_,  driver_ , pid,  profileGen );


void setup() {
    // Start serial communication
    Serial.begin(115200);

    encoder_.begin();

    pid->init();

    Serial.printf("CPU frequency: %d MHz\n", ESP.getCpuFreqMHz());

    // Generate the S-curve profile
    float totalDistance = 3.0f;       // Example distance to travel
    float commandedVelocity = 1.4;   // Example commanded velocity

    driver_.init();

    controller_.init();
    
}

void loop() {



    // uint64_t before_ = micros();

    // encoder_.getAngle();

    // uint64_t after_ = micros();

    // Serial.printf("Microseconds: %llu \n",  after_ - before_);
   
    // driver_.setPWMDutyCycle(20.0,70.0,15.0);
    // Serial.printf("ControlValue: %f \n", controller_.getControlValue());
    // Serial.printf("Angle: %f  \n", controller_.getdutyA());

    controller_.setOmega(0.1);
    controller_.setControlValue(10);


    // Serial.printf("Elapsed time: %llu \n", controller_.gettmp());



    //  Serial.printf("%f , %f , %f \n" , controller_.getdutyA(), controller_.getdutyB(), controller_.getdutyC());
    // Serial.printf("dutyB: %f \n" , controller_.getdutyB());
    // Serial.printf("dutyC: %f \n" , controller_.getdutyC());
    // profileGen.generateScurveProfile(3, 1.4);
   
     delay(2000);


    // // Get the generated profiles
    // positionProfile = &profileGen.getPositionProfile();
    // velocityProfile = &profileGen.getVelocityProfile();

    // currentTime = &profileGen.getTimeProfile();

    // float currentAngle = 0.085;

    // float targetVelocity = profileGen.getVelocityForAngle(currentAngle);
    
    // Print the profiles for verification
    // Serial.println("Position Profile:");
    // for (size_t i = 0; i < positionProfile->size(); ++i) {
    //     Serial.printf("%f, %f, %f \n",currentTime->at(i), positionProfile->at(i),velocityProfile->at(i) );
    // }

    // Serial.printf("Duration: %f \n", targetVelocity);

    // Serial.println("\nVelocity Profile:");
    // for (size_t i = 0; i < velocityProfile->size(); ++i) {
    //     Serial.printf("Step %d: %f\n", i, velocityProfile->at(i));
    // }


    //  Serial.println("Time Profile:");
    // for (size_t i = 0; i < currentTime->size(); ++i) {
    //     Serial.printf("Step %d: %f\n", i, currentTime->at(i));
    // }

    // Serial.println("\nTesting profile in loop...");


    // delay(1000);
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
