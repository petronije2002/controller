// Controller.h
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "AS5048my.h"  // Include your AS5048 encoder header
#include "Driver.h"    // Include your Driver header
#include "Algorithm.h" // Include your Algorithm base class
#include "ProfileGenerator.h"

class Controller
{
public:
    Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen);


    void init();

    void update(float desiredAngle); // Update method to perform control logic
    float constrain_(float value, float minValue, float maxValue);
    void commandTargetPosition(float targetPosition, float totalDistance);

    float getControlValue();

    float getdutyA();
    float getdutyB();
    float getdutyC();

    float omega = 2;

    void setOmega(float omega_);

    int totalSamples = 0;

    static void pwmTask(void *pvParameters); // Task function
    void startTask();

private:
    AS5048 &_encoder;
    Driver &_driver;
    Algorithm *_algorithm; // Pointer to the algorithm (PID, fuzzy, etc.)
    float controlValue;    // Store the control output
    ProfileGenerator &_profileGen;

    float dutyA = 0;
    float dutyB = 0;
    float dutyC = 0;

    int counter = 0;

    SemaphoreHandle_t memMutex =  xSemaphoreCreateMutex();;
};

#endif // CONTROLLER_H
