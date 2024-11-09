#ifndef FUZZYALGORITHM_H
#define FUZZYALGORITHM_H

#include "Algorithm.h"

class FuzzyAlgorithm : public Algorithm {
public:
    FuzzyAlgorithm();
    void init() override; // Override init function
    float calculate(float setpoint, float measured_value) override; // Override calculate function

private:
    // Fuzzy logic variables and methods
};

#endif // FUZZYALGORITHM_H
