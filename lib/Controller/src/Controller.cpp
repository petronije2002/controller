// Controller.cpp
#include "Controller.h"
#include "AS5048my.h"
#include "math.h"
#include "esp_timer.h"

Controller::Controller(AS5048 &encoder, Driver &driver, Algorithm *algorithm, ProfileGenerator &profileGen)
    : _encoder(encoder), _driver(driver), _algorithm(algorithm), _profileGen(profileGen), controlValue(0) {}

void Controller::init()
{

    startTask();
}

void Controller::startTask()
{
    // Create a FreeRTOS task to update the angle in the background
    xTaskCreatePinnedToCore(pwmTask, "pwmTask", 2048, this, 1, NULL, 1);
}

void Controller::update(float desiredAngle) {}
// void Controller::update(float desiredAngle) {
//     // Read the current angle from the encoder
//     // float currentAngle = _encoder.getAngle();

//     // Generate angles vector from the profile generator
//     // std::vector<float> angles = _profileGen.getPositionProfile();

//     // // Get the interpolated position and velocity from the profile generator
//     // float positionValue = _profileGen.getInterpolatedValue(currentAngle, _profileGen.getPositionProfile(), angles);
//     // float velocityValue = _profileGen.getInterpolatedValue(currentAngle, _profileGen.getVelocityProfile(), angles);

//     // Use positionValue or velocityValue in your PID calculation
//     // controlValue = _algorithm->calculate(desiredAngle, currentAngle);

//     // controlValue = constrain(controlValue, -100.0f, 100.0f);

//     // Serial.printf("Control value: %f \n", controlValue);

//     // Calculate the duty cycles for the three phases based on the current angle and controlValue
//     // float theta = currentAngle * (M_PI / 180.0); // Convert to radians

//     // Combine controlValue with the sine wave generation for duty cycles

//     if (controlValue>0){

//         this->dutyA = abs(controlValue) *  sin(21* theta); // Scale by controlValue
//         this->dutyB = abs(controlValue) * sin(21*theta + (2 * M_PI / 3)); // 120 degrees phase shift

//         this-> dutyC = abs(controlValue) * sin(21*theta + (4 * M_PI / 3)); // 240 degrees phase shift

//         // this->dutyA = abs(controlValue) * (0.5f * (1 + sin(theta))); // Scale by controlValue
//         // this->dutyB = abs(controlValue) * (0.5f * (1 + sin(theta + (2 * M_PI / 3)))); // 120 degrees phase shift

//         // this-> dutyC = abs(controlValue) * (0.5f * (1 + sin(theta + (4 * M_PI / 3)))); // 240 degrees phase shift

//         // this->dutyC = 100 - (dutyA + dutyB);

//         // _driver.setPWMDutyCycle(dutyA, dutyB, dutyC);

//         // float dutyC = controlValue * (0.5f * (1 + sin(theta + (4 * M_PI / 3)))); // 240 degrees phase shift

//     }else{

//         this-> dutyA = abs(controlValue ) * sin(21*theta + (2 * M_PI / 3));
//         this-> dutyB = abs(controlValue) * sin(21*theta);// 120 degrees phase shift
//         this-> dutyC = abs(controlValue) * sin(21* theta + (4 * M_PI / 3)); // 240 degrees phase shift

//         // float dutyC = controlValue * (0.5f * (1 + sin(theta + (4 * M_PI / 3)))); // 240 degrees phase shift
//         // this-> dutyC = 100 - (dutyA + dutyB);

//         // _driver.setPWMDutyCycle(dutyA, dutyB, dutyC);

//     }

//     float totalDuty = abs(dutyA) + abs(dutyB) + abs(dutyC);

// // Normalize the duty cycles so their sum equals 100 (or 1.0 in normalized terms)
//         if (totalDuty > 0) {
//             dutyA = (abs(dutyA) / totalDuty) * 100;
//             dutyB = (abs(dutyB) / totalDuty) * 100;
//             dutyC = (abs(dutyC) / totalDuty) * 100;
//         }
//     // float dutyA = controlValue * (0.5f * (1 + sin(theta))); // Scale by controlValue
//     // float dutyB = controlValue * (0.5f * (1 + sin(theta + (2 * M_PI / 3)))); // 120 degrees phase shift
//     // float dutyC = controlValue * (0.5f * (1 + sin(theta + (4 * M_PI / 3)))); // 240 degrees phase shift

//     // Ensure duty cycles are within [0, 1]
//     // dutyA = constrain(dutyA, 0.0f, 1.0f)*100;
//     // dutyB = constrain(dutyB, 0.0f, 1.0f)*100;
//     // dutyC = constrain(dutyC, 0.0f, 1.0f)*100;

//     // Set the PWM values for each phase

//      _driver.setPWMDutyCycle(dutyA, dutyB, dutyC);

// }

void Controller::setOmega(float omega_)
{

    xSemaphoreTake(memMutex, portMAX_DELAY);

    this->omega = omega_;
    this->totalSamples = int(2 * M_PI * 20000 / this->omega);
    xSemaphoreGive(memMutex);
}

void Controller::pwmTask(void *pvParameters)
{
    Controller *controller_ = (Controller *)pvParameters;
    // sensor->lastTime = millis();

    const int64_t interval_us = 1000000 / 20000; // 50 microseconds

    while (true)
    {
        int64_t start_time = esp_timer_get_time();

        xSemaphoreTake(controller_->memMutex, portMAX_DELAY);

        float tempAngle = controller_->omega * (controller_->counter / 20000);

        // float tempAngle = controller_->omega * (controller_->counter / controller_->_driver.pwm_config.frequency);

        controller_->dutyA = 0.5 * (1 + sin(tempAngle)) * 100;

        controller_->dutyB = 0.5 * (1 + sin(tempAngle + 2 * M_PI / 3)) * 100;
        controller_->dutyC = 0.5 * (1 + sin(tempAngle + 4 * M_PI / 3)) * 100;

        controller_->_driver.setPWMDutyCycle(controller_->dutyA, controller_->dutyB, controller_->dutyC);

        controller_->counter++;

        if (controller_->counter > controller_->totalSamples)
        {

            controller_->counter = 0;
        }

        xSemaphoreGive(controller_->memMutex);

        //  vTaskDelay(pdMS_TO_TICKS(2));

        int64_t elapsed_time = esp_timer_get_time() - start_time;
        if (elapsed_time < interval_us)
        {
            ets_delay_us(interval_us - elapsed_time);
        }
    }
}

float Controller::getdutyA()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);

    float aaa = this->dutyA;
    xSemaphoreGive(memMutex);

    return aaa;
}
float Controller::getdutyB()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);

    float aaa = this->dutyB;
    xSemaphoreGive(memMutex);
    return aaa;
}
float Controller::getdutyC()
{
    xSemaphoreTake(memMutex, portMAX_DELAY);

    float aaa = this->dutyC;
    xSemaphoreGive(memMutex);
    return aaa;
}

float Controller::getControlValue()
{

    xSemaphoreTake(memMutex, portMAX_DELAY);

    float aaa = this->controlValue;
    xSemaphoreGive(memMutex);

    return aaa;
}

// void Controller::commandTargetPosition(float targetPosition, float totalDistance) {
//     // Generate the S-curve profile based on the desired target position
//     _profileGen.generateScurveProfile(totalDistance);
// }

float Controller::constrain_(float value, float minValue, float maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}
