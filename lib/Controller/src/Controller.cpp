// Controller.cpp
#include "Controller.h"
#include "AS5048my.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "Communicator2.h"

#include "freertos/queue.h"

#include "esp_mac.h"

// Constructor initializes member references and control value
Controller::Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen, QueueHandler &queueHandler_, Communicator2 &com_ , SemaphoreHandle_t mem_  )
    : _encoder(encoder), _driver(driver), _algorithm(algorithm), _profileGen(profileGen), controlValue(0), queueHandler(queueHandler_), comm_(com_), memMutex(mem_)

{
}

// Initialize the controller and start the FreeRTOS task
void Controller::init()
{
    startTask();
    delay(1);
    initSinTable();
    delay(1);
    // setOmega(1.0);
    // setControlValue(20.0);
}

void Controller::commandTarget(float targetPosition, float targetVelocity)
{

    this->targetPOSITION = targetPosition;
    this->targetVELOCITY = targetVelocity;

    float distanceToGo_ = targetPosition - this->_encoder.getAngle();

    this->_profileGen.generateScurveProfile(distanceToGo_, targetVelocity);
};

// Start a FreeRTOS task for managing PWM control logic
void Controller::startTask()
{

    this->comm_._serial.println("Intro to pwm task");
    TaskParameters *params = new TaskParameters(&queueHandler, &Controller::setOmega, &Controller::setControlValue, this);

    // TaskParams* params = new TaskParams{ this};  // Dynamically allocate

    BaseType_t result = xTaskCreatePinnedToCore(pwmTask, "pwmTask", 16384, this, 5, NULL, 1);

    // this->comm_._serial.printf("Result %i", result);
    // this->comm_._serial.printf("Result: %i", result);

    // if (result != pdPASS) {
    //         this->comm_._serial.println("Error creating pwmTask");
    //     }else{

    //         this->comm_._serial.println("pwm task started");


    //     }
     xTaskCreatePinnedToCore(receiveCommandTask, "receivingCommandTask", 8192, static_cast<void *>(params), 2, NULL, 0);

    // xTaskCreatePinnedToCore(serialTask, "serialTask", 2048, this, 1, NULL, 1);  // Add task for serial communication
    // this->setOmega(0);
}

// Placeholder for control update logic (not yet implemented)
void Controller::update(float desiredAngle)
{
}

void Controller::initSinTable()
{
    for (int i = 0; i < SIN_TABLE_SIZE; i++)
    {
        sinTable[i] = sinf((TWO_PI * i) / SIN_TABLE_SIZE);
    }
}

float Controller::getSinFromTable(float angle)
{
    // Normalize the angle to the range [0, 2*pi)
    while (angle < 0)
        angle += TWO_PI;
    while (angle >= TWO_PI)
        angle -= TWO_PI;

    float stepSize = TWO_PI / SIN_TABLE_SIZE;

    // Calculate the index in the table and the fractional part
    float indexFloat = angle / stepSize; // Scaled index
    int index = (int)indexFloat;         // Integer part
    float t = indexFloat - index;        // Fractional part for interpolation

    // Handle wrapping for the last index
    int nextIndex = (index + 1) % SIN_TABLE_SIZE;

    // Perform linear interpolation
    float sineValue = (1 - t) * sinTable[index] + t * sinTable[nextIndex];
    return sineValue;
}

// FreeRTOS task for generating and updating PWM signals - in background
void Controller::pwmTask(void *pvParameters)
{



    Controller* controller_ = static_cast<Controller*>(pvParameters);

    // controller_->comm_._serial.println("in the taks");
    // Controller *controller_ = (Controller *)pvParameters;
    const int64_t interval_us = 1000000 / 20000; // 50 microseconds interval for a 20kHz update rate
    float scaleControle =0;
    float tempAngle = 0;
    while (true)
    {

        int64_t start_time = esp_timer_get_time(); // Get the current time

        xSemaphoreTake(controller_->memMutex, portMAX_DELAY);

        // Calculate the current angle based on the counter and omega
        // actually, it should  be 2*PI*freq_mod/freq_carrier)*controller_->counter
        // but, since I will command omega [rad/ses] (my desired angular velocity),
        // i decided to use omega , cince omega=2*pi*f
        tempAngle = controller_->omega * (controller_->counter / 20000.0);

        // Generate duty cycles for the three phases based on the angle no matter what maxControlValue is
        // dutyA, dutyB and dutyC, will always be between 0 and 100
        // Whatever is returned from the algorithm , as a controlValue ,  will be rescaled from 0-50.

        scaleControle = controller_->controlValue / controller_->maxControlValue;

        controller_->dutyA = 50 + 50 * scaleControle * controller_->getSinFromTable(tempAngle);
        controller_->dutyB = 50 + 50 * scaleControle * controller_->getSinFromTable(tempAngle + 2 * M_PI / 3);
        controller_->dutyC = 50 + 50 * scaleControle * controller_->getSinFromTable(tempAngle + 4 * M_PI / 3);

        // Send the duty cycles to the driver
        controller_->_driver.setPWMDutyCycle(controller_->dutyA, controller_->dutyB, controller_->dutyC);

        controller_->counter++;

        // Reset the counter if it exceeds the total samples for one wave cycle
        if (controller_->counter > controller_->totalSamples)
        {
            controller_->counter = 0;
        }
        if (controller_->counter % 1000 == 0)
        {

            // HERE YOU NEED TO SEND EVENTUALLY SOME VALUES TO SERIAL PORT
            // xQueueSend(controller_->serialQueue, &controller_->dutyA, pdMS_TO_TICKS(0)); // No dynamic allocation
        }

        xSemaphoreGive(controller_->memMutex);

        int64_t elapsed_time = esp_timer_get_time() - start_time; // Measure elapsed time


            vTaskDelay(1);
        // if (elapsed_time < interval_us)
        // {
        //     // controller_->tmp = elapsed_time;
        //     ets_delay_us(interval_us - elapsed_time); // Delay to maintain the update rate
        // }else{
        //     controller_->comm_._serial.printf("elapsed: %llu", elapsed_time);

        // }

        
    }
}

// Retrieve the duty cycle for phase A
float Controller::getdutyA()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);
    float aaa = this->dutyA;
    xSemaphoreGive(memMutex);
    return aaa;
}

// Retrieve the duty cycle for phase B
float Controller::getdutyB()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);
    float aaa = this->dutyB;
    xSemaphoreGive(memMutex);
    return aaa;
}

// Retrieve the duty cycle for phase C
float Controller::getdutyC()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);
    float aaa = this->dutyC;
    xSemaphoreGive(memMutex);
    return aaa;
}

// Constrain a value within the specified range
float Controller::constrain_(float value, float minValue, float maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

/** Sets desired angular velocity [rad/sec]
 * @brief That value will be used in a controller task, to generate sinusoidal signals
 *
 */
void Controller::setOmega(float omega_)
{
    xSemaphoreTake(memMutex, portMAX_DELAY);

    if (omega_ > 0)
    {
        this->omega = omega_;
        this->totalSamples = int(2 * M_PI * 20000 / this->omega); // Calculate the number of samples for a full wave

        this->counter = 0;
    }
    else if (omega_ == 0)
    {
        this->_driver.setPWMDutyCycle(0, 0, 0);
        this->counter = 0;
    }

    xSemaphoreGive(memMutex);
}

/**
 * @brief Sets the PWM duty cycles for three phases.
 *
 * This function sets the controlValue. That value will be
 * first constrained between 0 and maxControlValue and used to
 * calculate duty cycles.
 *
 * controller_->dutyA = 50 + 50 * (controller_->controlValue / controller_->maxControlValue) * sin(tempAngle);

 * @param controlValue The control signal used to scale the sine wave.
 */

void Controller::setControlValue(float controlValue_)
{

    xSemaphoreTake(memMutex, portMAX_DELAY);
    controlValue = constrain_(controlValue_, 0, this->maxControlValue);
    xSemaphoreGive(memMutex);
}

// Get controlValue in a thread safe way

float Controller::getControlValue()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);
    float aaa = this->controlValue;
    xSemaphoreGive(memMutex);
    return aaa;
}

void Controller::setMaxXontrolValue(float maxControlValue_)
{
    xSemaphoreTake(memMutex, portMAX_DELAY);
    this->maxControlValue = maxControlValue_;
    xSemaphoreGive(memMutex);
}

float Controller::getMaxXontrolValue()
{
    float aaa;
    xSemaphoreTake(memMutex, portMAX_DELAY);
    aaa = this->maxControlValue;
    xSemaphoreGive(memMutex);

    return aaa;
}

void Controller::receiveCommandTask(void *pvParameters)
{

    // TaskParameters* params = static_cast<TaskParameters*>(pvParameters);

    // Access the queueHandler, setOmega, and setControlValue via params
    TaskParameters *params = static_cast<TaskParameters *>(pvParameters);

    // Access the queueHandler, setOmega, and setControlValue via params
    QueueHandler *queueHandler = params->queueHandler;
    void (Controller::*setOmega)(float) = params->setOmega;
    void (Controller::*setControlValue)(float) = params->setControlValue;
    Controller *controller = params->controller;

    // QueueHandler* queueHandler = &params.controller_in.queueHandler;

    Message receivedMessage = {};

    // (controller->comm_._serial.printf("QueueHandler address: %p\n", controller->queueHandler));

    // controller_->comm_._serial.printf("QueueHandler address: %p\n", controller->queueHandler);

    // controller->comm_._serial.println("Entering the task");
    while (true)
    {
        // Wait for a message from the command queue
        // if( params->queuHandler_in.commandQueue->receiveMessageFromQueue( &params->queuHandler_in->commandQueue, &receivedMessage, pdMS_TO_TICKS(1))) {
        // Process the received message

        if (controller->queueHandler.receiveMessageFromQueue(controller->queueHandler.commandQueue, &receivedMessage, pdMS_TO_TICKS(100)))
        {

            //  (controller->comm_._serial.printf("QueueHandler address: %p\n", controller->queueHandler));

            switch (receivedMessage.type)
            {
            case MESSAGE_TYPE_COMMAND:


                // (controller->comm_._serial.printf("Position: %.2f\n",receivedMessage.payload.commandData.position));
                // (controller->comm_._serial.printf("Velocity: %.2f\n",receivedMessage.payload.commandData.velocity));



                (controller->*params->setOmega)(receivedMessage.payload.commandData.velocity);        // Call setOmega on Controller
                (controller->*params->setControlValue)(receivedMessage.payload.commandData.position); // Call setControlValue on Controller

                // controller_->setOmega(receivedMessage.payload.commandData.velocity);
                // controller_->setControlValue(receivedMessage.payload.commandData.position);

                break;

            case MESSAGE_TYPE_FEEDBACK:
                // do something
                break;

            case MESSAGE_TYPE_ERROR:
                // do something
                break;

            default:
                // do something
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
