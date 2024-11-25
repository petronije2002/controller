#include "Communicator.h"

// Constructor
Communicator::Communicator(HardwareSerial& serial, Controller& controller, uint32_t baudRate)
    : _serial(serial), _controller(controller), _baudRate(baudRate), _inputBuffer("") {
    _mutex = xSemaphoreCreateMutex(); // Create the semaphore
}

// Initialize communication and start the background task
void Communicator::begin() {
    _serial.begin(115200);
    _serial.println("Communicator initialized!");

    // Create the FreeRTOS task
    xTaskCreate(
        taskFunction,            // Task function
        "CommunicatorTask",      // Task name
        1000,                    // Stack size
        this,                    // Task parameter (pass `this` pointer)
        1,                       // Task priority
        nullptr                  // Task handle (not needed here)
    );
}

// Static FreeRTOS task function
void Communicator::taskFunction(void* parameter) {
    Communicator* instance = static_cast<Communicator*>(parameter);
    while (true) {
        instance->process();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Process incoming Serial data
void Communicator::process() {
    while (_serial.available()) {
        char inChar = (char)_serial.read();
        if (inChar == '\n') {
            handleCommand(_inputBuffer);
            _inputBuffer = "";
        } else {
            _inputBuffer += inChar;
        }
    }
}

// Handle received commands
void Communicator::handleCommand( String& command) {
    // Strip leading/trailing spaces from the command
    // command.trim();

    

    // Check for position and velocity in the same line
    if (command.startsWith("P") && command.indexOf(" V") > 0) {  // Position and Velocity
        // Example: "P100.34 V12.44"
        int spaceIndex = command.indexOf(" V");  // Find position of " V"
        
        if (spaceIndex > 0) {
            // Extract position and velocity from the command string
            String positionStr = command.substring(1, spaceIndex);  // Extract the part after 'P'
            String velocityStr = command.substring(spaceIndex + 2);  // Extract the part after ' V'

            // Convert the extracted values to float
            float targetPosition = positionStr.toFloat();
            float targetVelocity = velocityStr.toFloat();

            if (targetPosition != 0.0f && positionStr != "0") {  // Check if position is valid
                _controller.commandTarget(targetPosition, targetVelocity);
                send("Position command received: " + String(targetPosition) + ", Velocity command received: " + String(targetVelocity));
            } else {
                send("Invalid position value.");
                return;
            }

           
        } else {
            send("Invalid command format. Expected 'P<position> V<velocity>'.");
        }
    }
    else {
        send("Unknown or malformed command: " + command);
    }
}

// Send data safely to the Serial port
void Communicator::send(const String& data) {
    if (xSemaphoreTake(_mutex, portMAX_DELAY)) {
        _serial.println(data);
        xSemaphoreGive(_mutex);
    }
}
