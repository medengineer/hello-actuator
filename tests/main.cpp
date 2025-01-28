#include "actuator.h"
#include <iostream>

static void printState(const Actuator& actuator) {
    std::cout << "State: ";
    switch (actuator.getState()) {
        case Actuator::State::Uninitialized:
            std::cout << "Uninitialized";
            break;
        case Actuator::State::Operational:
            std::cout << "Operational";
            break;
        case Actuator::State::Error:
            std::cout << "Error";
            break;
        case Actuator::State::Unknown:
            std::cout << "Unknown";
            break;
    }
    std::cout << std::endl;
}

int main() {
    Actuator actuator("calibration.csv");
    
    std::cout << "1. Initial state:" << std::endl;
    printState(actuator);

    std::cout << "\n2. Initializing actuator:" << std::endl;
    if (actuator.initialize()) {
        std::cout << "Initialization successful" << std::endl;
    }
    printState(actuator);

    std::cout << "\n3. Testing various current values:" << std::endl;
    double test_currents[] = {
        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.065, // valid state
        2.066,2.5, 3.0, 4.7 // invalid state (don't respond)
    };
    for (double current : test_currents) {
        if (actuator.setCurrent(current)) {
            std::cout << "Current: " << current 
                      << "A, Position: " << actuator.getPosition() 
                      << "mm" << std::endl;
        }
    }

    std::cout << "\n4. Testing error state with invalid current:" << std::endl;
    if (!actuator.setCurrent(-1.0)) {
        std::cout << "Setting negative current failed (expected)" << std::endl;
    }
    printState(actuator);

    std::cout << "\n5. Attempting to set current in error state:" << std::endl;
    if (!actuator.setCurrent(2.0)) {
        std::cout << "Setting current failed (expected)" << std::endl;
    }

    std::cout << "\n6. Shutting down:" << std::endl;
    actuator.shutdown();
    printState(actuator);

    return 0;
} 