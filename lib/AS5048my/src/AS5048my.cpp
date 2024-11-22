#include "AS5048my.h"


AS5048::AS5048(int csPin) : _csPin(csPin), angle(0.0), velocity(0.0)
{
    // Create a mutex for thread-safe access to shared variables
    angleMutex = xSemaphoreCreateMutex();
}

// Initializes the encoder
void AS5048::begin()
{
    // Configure the CS pin as output
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH); // Deselect the encoder initially
    
    // Initialize SPI communication
    SPI.begin();
    
    // Record the starting time
    lastTime = micros();

    // Start the background task to read angle and velocity
    startTask();

    // Initialize the previous angle
    previousAngle = getAngle();
}

// Creates and starts a FreeRTOS task for background angle updates
void AS5048::startTask()
{
    // Creates a task pinned to Core 0
    xTaskCreatePinnedToCore(angleTask, "AngleTask", 2048, this, 1, NULL, 0);
}

// Static task function for updating the angle in the background
void AS5048::angleTask(void *pvParameters)
{
    // Cast the parameter to the AS5048 object
    AS5048 *sensor = (AS5048 *)pvParameters;
    sensor->lastTime = micros();

    while (true)
    {
        // Read the current angle and update velocity
        sensor->readAngle();

        // Calculate the elapsed time since the last reading
        unsigned long timeDiff = sensor->safeMicros(sensor->lastTime);

        if (timeDiff > 0)
        {
            // Ensure thread-safe access to shared variables
            xSemaphoreTake(sensor->angleMutex, portMAX_DELAY);

            // Calculate the difference in angle
            float angleDifference = sensor->angle - sensor->previousAngle;

            // Handle angle wrapping (e.g., crossing 0 or 360 degrees)
            if (angleDifference > 180.0f)
                angleDifference -= 360.0f; // Wrap down
            else if (angleDifference < -180.0f)
                angleDifference += 360.0f; // Wrap up

            // Calculate velocity in degrees per second
            sensor->velocity = 1000 * angleDifference / timeDiff;

            // Update the previous angle and time
            sensor->previousAngle = sensor->angle;
            sensor->lastTime = micros();

            // Release the mutex
            xSemaphoreGive(sensor->angleMutex);
        }

        // Delay to control the task's update rate (2 ms)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Reads the raw angle from the encoder and updates the `angle` variable
void AS5048::readAngle()
{
    uint16_t rawAngle;        // Raw angle value from the encoder
    uint16_t command = 0x3FFF; // Command to read the angle

    // Begin SPI communication with the encoder
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    digitalWrite(_csPin, LOW);       // Select the encoder
    SPI.transfer16(command);        // Send the read command
    digitalWrite(_csPin, HIGH);     // Deselect the encoder

    digitalWrite(_csPin, LOW);      // Select again to receive data
    rawAngle = SPI.transfer16(0x0000); // Read the response
    digitalWrite(_csPin, HIGH);     // Deselect the encoder
    SPI.endTransaction();

    // Mask the top 2 bits (PAR and EF) and convert to degrees
    rawAngle &= 0x3FFF;
    float newAngle = (float)rawAngle / 16384.0 * 360.0;

    // Update the shared `angle` variable safely
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    angle = newAngle;
    xSemaphoreGive(angleMutex);
}

// Returns the current angle (thread-safe)
float AS5048::getAngle()
{
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    float currentAngle = angle;
    xSemaphoreGive(angleMutex);
    return currentAngle;
}

// Returns the current velocity (thread-safe)
float AS5048::getVelocity()
{
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    float currentVelocity = velocity;
    xSemaphoreGive(angleMutex);
    return currentVelocity;
}

// Handles micros() overflow safely
unsigned long AS5048::safeMicros(unsigned long lastTime)
{
    unsigned long currentTime = micros();
    if (currentTime < lastTime)
    {
        // Handle overflow: current time wrapped around
        return (0xFFFFFFFF - lastTime) + currentTime;
    }
    else
    {
        // No overflow: regular difference
        return currentTime - lastTime;
    }
}
