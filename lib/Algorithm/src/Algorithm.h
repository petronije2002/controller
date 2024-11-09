#ifndef ALGORITHM_H
#define ALGORITHM_H

class Algorithm {
public:
    virtual void init() = 0; // Pure virtual function for initialization
    virtual float calculate(float setpoint, float measured_value) = 0; // Pure virtual function for calculation
    virtual ~Algorithm() {} // Virtual destructor
};

#endif // ALGORITHM_H
