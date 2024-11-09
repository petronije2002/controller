#include <Arduino.h>
#include "AS5048my.h"

AS5048 encoder(5);  // Replace with the actual CS pin

void setup() {
    Serial.begin(115200);

   
    encoder.begin();
    encoder.startTask();
    
}

void loop() {
    // Access the angle and velocity from the main loop
    float angle = encoder.getAngle();
    float velocity = encoder.getVelocity();

    Serial.print("Angle: ");
    Serial.println(angle);
    Serial.print("Velocity: ");
    Serial.println(velocity);

    delay(100); // Adjust as needed
}



// Use this only as reminder :



// const int csPin = 5;  // CSn pin connected to GPIO 5
// uint16_t readAngle();  // Function declaration
// float degAngle = 0; //Angle in degrees
// uint16_t command = 0b1111111111111111; //read command (0xFFF)

// uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)


// void setup() {
//   Serial.begin(115200);
//   SPI.begin();

  
//   pinMode(csPin, OUTPUT);
//   digitalWrite(csPin, HIGH);  // Deselect AS5048A initially
// }

// void loop() {
//   // Read angle from AS5048A
//   uint16_t angle = readAngle();
  
//   // Print the angle value
//   Serial.print("Angle: ");
//   Serial.println(degAngle,4);

//   delay(500);  // Wait for 1 second
// }


// // uint16_t readAngle() {
// //   // Begin SPI transaction with settings
// //   SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE1));

// //   // No need to toggle csPin since it is already held low (always active)
  
// //   // Send the command to read angle (address 0x3FFF)
// //   SPI.transfer16(0x3FFF);

// //   // Read the response (16-bit angle value)
// //   uint16_t angle = SPI.transfer16(0x0000);

// //   // End the SPI transaction
// //   SPI.endTransaction();

// //   // Mask the result to remove the top 2 bits (PAR and EF)
// //   angle = angle & 0b0011111111111111;

// //   // Convert to degrees
// //   float degAngle = (float)angle / 16384.0 * 360.0; // 16384 = 2^14, 360 = 360 degrees

// //   return degAngle;
// // }

// uint16_t readAngle() {

//   SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE1));

//   digitalWrite(csPin, LOW);  // Select AS5048A


//   // Send the command to read angle (address 0x3FFF)
//   SPI.transfer16(0x3FFF);


//   digitalWrite(csPin, HIGH);

//   delay(10);

//   digitalWrite(csPin, LOW);

//   // rawData = SPI.transfer16(command);

//   // Read the response (16-bit angle value)
//   uint16_t angle = SPI.transfer16(0x0000);

//   digitalWrite(csPin, HIGH);  // Deselect AS5048A
//   SPI.endTransaction();

//   angle = angle & 0b0011111111111111; //removing the top 2 bits (PAR and EF)


//     ////////////


//   degAngle = (float)angle / 16384.0 * 360.0; //16384 = 2^14, 360 = 360 degrees

//   return degAngle;
// }
