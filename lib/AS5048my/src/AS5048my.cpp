#include "AS5048my.h"

AS5048::AS5048(int csPin) : _csPin(csPin), angle(0.0), velocity(0.0) {
    angleMutex = xSemaphoreCreateMutex();
}



void AS5048::begin() {
    // Change
    // add comment to push it .. push it to commeit
    // more comments
    pinMode(_csPin, OUTPUT);
    // digitalWrite(_csPin, LOW);  // Deselect initially
    SPI.begin();
    lastTime = micros();

    previousAngle = getAngle();
}

void AS5048::startTask() {
    // Create a FreeRTOS task to update the angle in the background
    xTaskCreatePinnedToCore(angleTask, "AngleTask", 2048, this, 1, NULL, 1);
}

void AS5048::angleTask(void *pvParameters) {
    AS5048 *sensor = (AS5048 *)pvParameters;
    sensor->lastTime = millis();

    while (true) {

        sensor->readAngle();
        
        // Calculate velocity based on the time elapsed
        // unsigned long currentTime = millis();

        unsigned long timeDiff = sensor->safeMicros(sensor->lastTime);

        if (timeDiff > 0) {
            xSemaphoreTake(sensor->angleMutex, portMAX_DELAY);

            float angleDifference = sensor->angle - sensor->previousAngle;

            // Handle angle wrapping
            if (angleDifference > 180.0f) {
                angleDifference -= 360.0f;  // Wrap down
            } else if (angleDifference < -180.0f) {
                angleDifference += 360.0f;  // Wrap up
            }

            sensor->velocity = 1000* angleDifference / (timeDiff ); // Degrees per second

            sensor->previousAngle = sensor->angle;
            sensor->lastTime = millis();

            xSemaphoreGive(sensor->angleMutex);
           
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay based on desired update rate
    }
}

void AS5048::readAngle() {
    uint16_t rawAngle;
    uint16_t command = 0x3FFF;  // Read command to get angle

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    digitalWrite(_csPin, LOW);      // Select the encoder
    SPI.transfer16(command);        // Send the command to read angle

    digitalWrite(_csPin, HIGH);

    delay(2);

    digitalWrite(_csPin, LOW);

    // Read the response and store it in rawAngle
    rawAngle = SPI.transfer16(0x0000);

    digitalWrite(_csPin, HIGH);     // Deselect the encoder
    SPI.endTransaction();

    
    // Mask the top 2 bits (PAR and EF) and convert to degrees
    rawAngle &= 0x3FFF;
    float newAngle = (float)rawAngle / 16384.0 * 360.0;

    // Update the angle with mutex for thread safety
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    angle = newAngle;
    xSemaphoreGive(angleMutex);
}

float AS5048::getAngle() {
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    float currentAngle = angle;
    xSemaphoreGive(angleMutex);
    return currentAngle;
}

float AS5048::getVelocity(){

    xSemaphoreTake(angleMutex, portMAX_DELAY);
    float currentVelocity = velocity;
    xSemaphoreGive(angleMutex);
    return currentVelocity;


}







unsigned long AS5048::safeMicros(unsigned long lastTime) {
    unsigned long currentTime = millis();
    if (currentTime < lastTime) {
        // Overflow has occurred, correct the time difference
        return (0xFFFFFFFF - lastTime) + currentTime;
    } else {
        // No overflow, regular time difference
        return currentTime - lastTime;
    }
}