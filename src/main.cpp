// #ifndef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 
// #define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 240
// #endif

#include <Arduino.h>
#include "freertosconfig.h"

#include "ProfileGenerator.h"


#include "AS5048my.h"
// #include "Driver.h"
#include "PIDAlgorithm.h" 
// #include "esp_mac.h"
// #include "string.h"
#include "Controller.h"
#include "Communicator2.h"
#include "QueueHandler.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// Buffer to hold task information



QueueHandler queueHandler= QueueHandler();



ProfileGenerator profileGen(5, 1, 20);

AS5048 encoder_(5);

Driver driver_(13,15,12,14,27,26);

Algorithm* pid = new PIDController(1.0f, 0.1f, 0.05f); // Create a pointer to PIDController


Communicator2 communicator_ = Communicator2( queueHandler);

SemaphoreHandle_t mem_ = xSemaphoreCreateMutex();

Controller controller_( encoder_,  driver_ , pid,  profileGen, queueHandler,  communicator_,   mem_);

// Communicator2 communicator = Communicator2();




void setup() {

    // Serial.begin(115200);


    

    encoder_.begin();
    driver_.init();

    controller_.init();

    encoder_.resetMultiTurnAngle();

   
    

   

}


void loop(){

    



    // controller_.setControlValue(20);
    // controller_.setOmega(1);
    

    // // Serial.printf("EncoderAngle: %f\n",  encoder_.getMultiTurnAngle());
    
    // delay(2000);

}
