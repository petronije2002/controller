#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Controller.h" // Include the Controller class

class Communicator {
public:
    Communicator(HardwareSerial& serial, Controller& controller, uint32_t baudRate = 9600);
    void begin();                // Initialize communication and start the task
    void send(const String& data); // Send data to the Serial port

private:
    HardwareSerial& _serial;      // Reference to the Serial object
    Controller& _controller;      // Reference to the Controller object
    uint32_t _baudRate;           // Baud rate for Serial communication
    String _inputBuffer;          // Buffer to store incoming data

    SemaphoreHandle_t _mutex;     // Semaphore for thread-safe access

    static void taskFunction(void* parameter); // FreeRTOS task function
    void process();               // Internal method to process Serial input
    void handleCommand(String& command); // Process incoming commands
};


#endif
