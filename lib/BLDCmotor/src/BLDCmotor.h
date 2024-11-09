#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    float resistance;         // Winding resistance in Ohms
    float inductance;         // Winding inductance in Henrys
    float voltage;            // Phase voltage in Volts
    float current;            // Motor phase current in Amps
    float angularVelocity;    // Electrical angular velocity (rad/s)
    int polePairs;            // Number of pole pairs
    float Kv;                 // Velocity constant (rad/s per Volt)
    float maxCurrent;         // Maximum current rating (A)
    float maxVoltage;         // Maximum voltage rating (V)
    float currentLimit;       // Current limit for operation (A)
    float voltageLimit;       // Voltage limit for operation (V)

  public:
    // Constructor with additional parameters
    Motor(int poles, float kvRating, float maxCurrent, float maxVoltage);
    
    // Set winding parameters
    void setWindingParameters(float res, float ind);
    
    // Set current
    void setCurrent(float motorCurrent);
    
    // Set voltage
    void setVoltage(float motorVoltage);
    
    // Set angular velocity (electrical)
    void setAngularVelocity(float omega);
    
    // Calculate flux linkage: λ = (V - I * R) / ω
    float calculateFluxLinkage();
    
    // Calculate back EMF using Kv: E_b = ω / Kv
    float calculateBackEMF();
    
    // Calculate torque: T = (3/2) * (P/2) * λ * I
    float calculateTorque();
    
    // Get Kv
    float getKv();
    
    // Additional getters for max current and max voltage
    float getMaxCurrent();
    float getMaxVoltage();
    
    // Additional setters for current and voltage limits if needed
    void setCurrentLimit(float limit);
    void setVoltageLimit(float limit);
};

#endif // MOTOR_H
