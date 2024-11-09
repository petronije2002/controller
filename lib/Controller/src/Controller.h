// Controller.h
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "AS5048my.h"  // Include your AS5048 encoder header
#include "Driver.h"    // Include your Driver header
#include "Algorithm.h" // Include your Algorithm base class
#include "ProfileGenerator.h"


class Controller {
public:
    Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen);
    void update(float desiredAngle); // Update method to perform control logic
    float constrain(float value, float minValue, float maxValue) ;
    void commandTargetPosition(float targetPosition, float totalDistance);




private:
    AS5048 &_encoder;
    Driver &_driver;
    Algorithm *_algorithm; // Pointer to the algorithm (PID, fuzzy, etc.)
    float controlValue;    // Store the control output
    ProfileGenerator &_profileGen;



  
};

#endif // CONTROLLER_H
