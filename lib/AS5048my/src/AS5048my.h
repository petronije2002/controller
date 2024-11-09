#ifndef AS5048MY_H
#define AS5048MY_H

#include <Arduino.h>
#include <SPI.h>

class AS5048
{
public:
    AS5048(int csPin);
    void begin();
    void startTask();    // Start the background task
    float getAngle();    // Retrieve the latest angle value
    float getVelocity(); // Retrieve the latest velocity value

private:
    int _csPin;
    float angle; // Current angle in degrees
    float previousAngle;
    float velocity; // Velocity in degrees per second
    unsigned long lastTime;
    SemaphoreHandle_t angleMutex;              // Mutex for thread-safe access to angle
    static void angleTask(void *pvParameters); // Task function
    void readAngle();                          // Reads and updates angle and velocity
    unsigned long safeMicros(unsigned long lastTime);
};

#endif
