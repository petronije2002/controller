// Controller.h
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "AS5048my.h"  // Include your AS5048 encoder header
#include "Driver.h"    // Include your Driver header
#include "Algorithm.h" // Include your Algorithm base class
#include "ProfileGenerator.h"
#define QUEUE_SIZE 100

// Controller class for managing control logic of a motor
class Controller
{
public:
    Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen);

    void init(); // Initialize the controller and start the control task

    void update(float desiredAngle); // Update method for control logic
    float constrain_(float value, float minValue, float maxValue); // Constrain a value between specified limits
    void commandTargetPosition(float targetPosition, float totalDistance); // Command the motor to move to a target position with a specified distance

    float getControlValue(); // Get the current control value
    float getdutyA();        // Get the duty cycle for phase A
    float getdutyB();        // Get the duty cycle for phase B
    float getdutyC();        // Get the duty cycle for phase C

    float omega = 2; // Angular velocity parameter
    float maxControlValue = 50;

    void setOmega(float omega_); // Set the angular velocity and update related parameters
    // void addToBuffer(float value) ;
    // bool getFromBuffer(float *value) ;



    // int64_t tmp =0;
    int64_t gettmp();



    int totalSamples = 0; // Total samples for one full wave cycle

    static void pwmTask(void *pvParameters); // Task function for PWM generation
    static void serialTask(void *pvParameters);
    
    void startTask(); // Start the FreeRTOS task for PWM control

    void setControlValue(float controlValue_);


   


private:
    AS5048 &_encoder;             // Reference to the encoder object
    Driver &_driver;              // Reference to the driver object
    Algorithm *_algorithm;        // Pointer to the control algorithm (e.g., PID)
    ProfileGenerator &_profileGen; // Reference to the profile generator

    float dutyA = 0; // Duty cycle for phase A
    float dutyB = 0; // Duty cycle for phase B
    float dutyC = 0; // Duty cycle for phase C

    float controlValue = 0;

    int counter = 0; // Counter for time step increments

    float queueStorage[QUEUE_SIZE];
    StaticQueue_t queueControlBlock;
    QueueHandle_t serialQueue;

    SemaphoreHandle_t memMutex = xSemaphoreCreateMutex(); // Mutex for thread-safe access
};

#endif // CONTROLLER_H
