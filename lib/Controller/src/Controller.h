// Controller.h
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "AS5048my.h"  // Include your AS5048 encoder header
#include "Driver.h"    // Include your Driver header
#include "Algorithm.h" // Include your Algorithm base class
#include "ProfileGenerator.h"
#include "freertos/FreeRTOS.h"

#include "freertos/queue.h"
#include "QueueHandler.h"
#include "Communicator2.h"

#define QUEUE_SIZE 20
#define SIN_TABLE_SIZE 512 

// Controller class for managing control logic of a motor
class Controller
{
public:
    Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen,QueueHandler &queueHandler, Communicator2 &com_,SemaphoreHandle_t mem_);

    void init(); // Initialize the controller and start the control task

    void update(float desiredAngle); // Update method for control logic
    float constrain_(float value, float minValue, float maxValue); // Constrain a value between specified limits
    void commandTarget(float targetPosition, float totalDistance); // Command the motor to move to a target position with a specified distance

    float getControlValue(); // Get the current control value
    float getdutyA();        // Get the duty cycle for phase A
    float getdutyB();        // Get the duty cycle for phase B
    float getdutyC();        // Get the duty cycle for phase C

    float omega; // Angular velocity parameter

    void setOmega(float omega_); // Set the angular velocity and update related parameters
    


    float targetPOSITION;
    float targetVELOCITY;



    int totalSamples = 0; // Total samples for one full wave cycle

    static void pwmTask(void *pvParameters); // Task function for PWM generation
    static void receiveCommandTask(void *pvParameters);
    // static void serialFeedbackTask(void *pvParameters);
    
    void startTask(); // Start the FreeRTOS task for PWM control

    void setControlValue(float controlValue_);
    void setMaxXontrolValue(float maxControlValue_);
    float getMaxXontrolValue();
    

    void initSinTable();
    float getSinFromTable(float angle) ;
    AS5048 &_encoder;             // Reference to the encoder object
    QueueHandler &queueHandler;

   struct TaskParameters {
        QueueHandler* queueHandler;  // Pointer to QueueHandler
        void (Controller::*setOmega)(float);  // Pointer to Controller's setOmega method
        void (Controller::*setControlValue)(float);  // Pointer to Controller's setControlValue method
        Controller* controller;
        // Constructor to initialize the members
        TaskParameters(QueueHandler* qh, void (Controller::*omega)(float), void (Controller::*control)(float),Controller* ctrl) 
            : queueHandler(qh), setOmega(omega), setControlValue(control), controller(ctrl) {}
    };

    Communicator2 &comm_;


private:
    
    Driver &_driver;              // Reference to the driver object
    Algorithm *_algorithm;        // Pointer to the control algorithm (e.g., PID)
    ProfileGenerator &_profileGen; // Reference to the profile generator

    float dutyA = 0; // Duty cycle for phase A
    float dutyB = 0; // Duty cycle for phase B
    float dutyC = 0; // Duty cycle for phase C

    float controlValue = 0;

    float maxControlValue = 50;

    int counter = 0; // Counter for time step increments

    float sinTable[SIN_TABLE_SIZE];

    
    
    SemaphoreHandle_t memMutex ;

    
    

    // Mutex for thread-safe access
};

#endif // CONTROLLER_H
