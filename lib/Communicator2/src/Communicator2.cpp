#include "Communicator2.h"
#include <Arduino.h>  
#include "QueueHandler.h"
#include "MessageDefinitions.h"


// Constructor: Initializes message queues and starts the task

Communicator2::Communicator2(QueueHandler &queuhandler_):queueHandler(queuhandler_) {

   

    // this->_serial.begin(115200);
    USBSerial.begin(115200);
    // Create the Communicator task that listens for incoming messages
    // xTaskCreate(CommunicatorTask, "CommunicatorTask", 4096, this, 1, &commTaskHandle);
    xTaskCreatePinnedToCore(CommunicatorTask, "receivingCommandTask", 4096, this, 2, NULL, 0);


}

// Destructor: Clean up resources
Communicator2::~Communicator2() {
    // Delete the FreeRTOS task when it's no longer needed
    // vTaskDelete(this->commTaskHandle);
    
    // Optionally, delete queues if you want to clean up explicitly (not necessary if queues are used only within the task)
//     vQueueDelete(queueHandler.commandQueue);
//    vQueueDelete(queueHandler.feedbackQueue);
//    vQueueDelete(queueHandler.errorQueue);
//    vQueueDelete(queueHandler.toSerialQueue);
   
}

 

// Static task function that runs in the background
void Communicator2::CommunicatorTask(void* pvParameters) {
    // Cast the parameter back to the Communicator object
    Communicator2* communicator = (Communicator2*)pvParameters;

    


    while (1) {
        communicator->handleSerialInput();  // Handle serial input in a loop
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay to avoid CPU hogging, gives other tasks a chance
    }
}

// Method to handle serial input and process the messages
void Communicator2::handleSerialInput() {
    // Check if there is any data available on the serial port
    if (USBSerial.available() > 0) {
        String command = USBSerial.readStringUntil('\n');  // Read until newline

        // this->_serial.printf("received: %s\n", command.c_str());
        Message msg={};
        parseCommand(command, msg);  // Parse the command into a Message struct

        
        
        if(queueHandler.sendMessageToQueue(queueHandler.commandQueue, msg)){

            USBSerial.printf("Position: %.2f\n", msg.payload.commandData.position);
            USBSerial.printf("Velocity: %.2f\n", msg.payload.commandData.velocity);
        };  // Send the parsed message to the appropriate queue
    }
}

// Method to parse the command string and populate the Message struct
void Communicator2::parseCommand(const String& command, Message &msg) {
    char type = command.charAt(0);  // Get the message type (e.g., 'C', 'F', 'E')
    msg.id = command.substring(1, 2).toInt();  // Get the message ID (e.g., 1, 2, 3)

    // Depending on the message type, parse the data accordingly
    switch (type) {
        case 'C':  // Command message (e.g., "C1 P110.2 V2.32")

            // this->_serial.println("It was C type command");
            msg.type = MESSAGE_TYPE_COMMAND;
            sscanf(command.c_str(), "C%*d P%f V%f", 
                   &msg.payload.commandData.position, 
                   &msg.payload.commandData.velocity);

                //    this->_serial.println("Command received");
                //    this->_serial.printf("QueueHandler address: %p\n", queueHandler);

            break;
        case 'F':  // Feedback message (e.g., "F2 P50.1 V1.5 T3.2")
            msg.type = MESSAGE_TYPE_FEEDBACK;
            sscanf(command.c_str(), "F%*d P%f V%f T%f",
                   &msg.payload.feedbackData.currentPosition,
                   &msg.payload.feedbackData.currentVelocity,
                   &msg.payload.feedbackData.torque);
            break;
        case 'E':  // Error message (e.g., "E3 404 'Not Found'")
            msg.type = MESSAGE_TYPE_ERROR;
            sscanf(command.c_str(), "E%*d %d '%[^\']'",
                   &msg.payload.errorData.errorCode,
                   &msg.payload.errorData.errorMessage);
            break;
        default:
            USBSerial.println("Unknown message type, message ignored!");
            break;
    }
}
