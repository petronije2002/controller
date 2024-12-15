#ifndef COMMUNICATOR2_H
#define COMMUNICATOR2_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"             // FreeRTOS task management
#include  "freertos/queue.h"  
#include "QueueHandler.h"

// #include <freertos/semphr.h>


// Communicator class declaration
class Communicator2 {
public:
    // Constructor: Initializes queues and starts the task
    Communicator2(QueueHandler &queuhandler);

    // Destructor: Clean up any resources (e.g., FreeRTOS tasks, queues)
    ~Communicator2();

    // Method to parse incoming serial messages
    void handleSerialInput();
    void initializeSerial(uint32_t baudRate);
    // HardwareSerial  _serial = HardwareSerial(0); 

private:
    // FreeRTOS queue handles for different message types
    // QueueHandle_t commandQueue;
    // QueueHandle_t feedbackQueue;
    // QueueHandle_t errorQueue;

    QueueHandler &queueHandler;

    

    // Handle for the communicator task
    TaskHandle_t commTaskHandle;

    // Static task function that runs in the background
    static void CommunicatorTask(void* pvParameters);

    // Method to parse a received command string and fill a Message struct
    void parseCommand(const String& command, Message &msg);

    // Method to send a parsed message to the appropriate queue
    // void sendMessageToQueue(Message &msg);
};

#endif // COMMUNICATOR_H
