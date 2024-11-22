// Controller.cpp
#include "Controller.h"
#include "AS5048my.h"
#include "math.h"
#include "esp_timer.h"

float preallocBuffer[QUEUE_SIZE];
volatile int head = 0;
volatile int tail = 0;

// Constructor initializes member references and control value
Controller::Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen)
    : _encoder(encoder), _driver(driver), _algorithm(algorithm), _profileGen(profileGen), controlValue(0) {
        serialQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(float), (uint8_t *)queueStorage, &queueControlBlock);
; 

    }

// Initialize the controller and start the FreeRTOS task
void Controller::init()
{
    startTask();
}



// Start a FreeRTOS task for managing PWM control logic
void Controller::startTask()
{
    xTaskCreatePinnedToCore(pwmTask, "pwmTask", 2048, this, 1, NULL, 1);
    // xTaskCreatePinnedToCore(serialTask, "serialTask", 2048, this, 3, NULL, 1);  // Add task for serial communication

}

// Placeholder for control update logic (not yet implemented)
void Controller::update(float desiredAngle) {}



// FreeRTOS task for generating and updating PWM signals - in background
void Controller::pwmTask(void *pvParameters)
{
    Controller *controller_ = (Controller *)pvParameters;
    const int64_t interval_us = 1000000 / 20000; // 50 microseconds interval for a 20kHz update rate

    while (true)
    {
        int64_t start_time = esp_timer_get_time(); // Get the current time

        xSemaphoreTake(controller_->memMutex, portMAX_DELAY);

        // Calculate the current angle based on the counter and omega
        float tempAngle = controller_->omega * (controller_->counter /20000.0);

        // Generate duty cycles for the three phases based on the angle no matter what maxControlValue is
        // dutyA, dutyB and dutyC, will always be between 0 and 100
        // Whatever is returned from the algorithm , as a controlValue ,  will be rescaled from 0-50.

        controller_->dutyA = 50 + 50 * (controller_->controlValue / controller_->maxControlValue) * sin(tempAngle);
        controller_->dutyB = 50 + 50 * (controller_->controlValue / controller_->maxControlValue) * sin(tempAngle + 2 * M_PI / 3);
        controller_->dutyC = 50 + 50 * (controller_->controlValue / controller_->maxControlValue) * sin(tempAngle + 4 * M_PI / 3);

        // Send the duty cycles to the driver
        controller_->_driver.setPWMDutyCycle(controller_->dutyA, controller_->dutyB, controller_->dutyC);

        controller_->counter++;

        // Reset the counter if it exceeds the total samples for one wave cycle
        if (controller_->counter > controller_->totalSamples)
        {
            controller_->counter = 0;
        }
        if(controller_->counter %1000 ==0){
                   xQueueSend(controller_->serialQueue, &controller_->dutyA, pdMS_TO_TICKS(1)); // No dynamic allocation
        }

        xSemaphoreGive(controller_->memMutex);

        int64_t elapsed_time = esp_timer_get_time() - start_time; // Measure elapsed time


        if (elapsed_time < interval_us)
        {
            // controller_->tmp = elapsed_time;



            ets_delay_us(interval_us - elapsed_time); // Delay to maintain the update rate
        }
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
    this->omega = omega_;
    this->totalSamples = int(2 * M_PI * 20000 / this->omega); // Calculate the number of samples for a full wave
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



// int64_t Controller::gettmp()
// {
//     xSemaphoreTake(memMutex, portMAX_DELAY);
//     float aaa = this->tmp;
//     xSemaphoreGive(memMutex);
//     return aaa;
// }

void Controller::serialTask(void *pvParameters)
{
    Controller *controller_ = (Controller *)pvParameters;

    while (true)
    {
        float receivedValue;
        if (xQueueReceive(controller_->serialQueue, &receivedValue, pdMS_TO_TICKS(1)) == pdPASS)
        {
            Serial.println(receivedValue,4);
        }
    }
}

// void Controller::addToBuffer(float value) {
//     int nextHead = (head + 1) % QUEUE_SIZE;

//     // Check for buffer overrun
//     if (nextHead != tail) {
//         preallocBuffer[head] = value;
//         head = nextHead;
//     }
// }

// bool Controller::getFromBuffer(float *value) {
//     if (head == tail) {
//         return false;  // Buffer is empty
//     } else {
//         *value = preallocBuffer[tail];
//         tail = (tail + 1) % QUEUE_SIZE;
//         return true;
//     }
// }
