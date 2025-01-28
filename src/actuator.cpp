#include "actuator.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

Actuator::Actuator(const std::string& calibration_file) 
    : current_current_(0.0)
    , current_state_(State::Uninitialized) {
    if (!loadCalibrationData(calibration_file)) {
        current_state_ = State::Error;
    }
}

bool Actuator::initialize() {
    if (current_state_ == State::Error) {
        if (!loadCalibrationData("data/calibration.csv")) {
            return false;
        }
    }
    current_current_ = 0.0;
    current_state_ = State::Operational;
    return true;
}

bool Actuator::setCurrent(double current) {
    std::cout << "Setting current to: " << current << std::endl;
    
    if (current_state_ != State::Operational) {
        std::cout << "Failed to set current: actuator not operational (state=" 
                  << static_cast<int>(current_state_) << ")" << std::endl;
        return false;
    }

    if (!isCurrentValid(current)) {
        std::cout << "Failed to set current: invalid current value" << std::endl;
        current_state_ = State::Error;
        return false;
    }

    current_current_ = current;
    std::cout << "Successfully set current to: " << current << std::endl;
    return true;
}

double Actuator::getPosition() const {
    if (current_state_ != State::Operational) {
        return 0.0;
    }
    return findNearestPosition(current_current_);
}

Actuator::State Actuator::getState() const {
    return current_state_;
}

bool Actuator::shutdown() {
    current_current_ = 0.0;
    if (current_state_ != State::Error) {
        current_state_ = State::Uninitialized;
    }
    return true;
}

bool Actuator::loadCalibrationData(const std::string& filename) {
    std::cout << "Attempting to load calibration file: " << filename << std::endl;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open calibration file" << std::endl;
        return false;
    }

    calibration_data_.clear();
    std::string line;
    
    // Skip header if present
    std::getline(file, line);
    std::cout << "Header line: " << line << std::endl;

    int line_count = 0;
    while (std::getline(file, line)) {
        line_count++;
        std::stringstream ss(line);
        std::string current_str, position_str;
        
        if (std::getline(ss, current_str, ',') && 
            std::getline(ss, position_str, ',')) {
            try {
                double current = std::stod(current_str);
                double position = std::stod(position_str);
                calibration_data_.emplace_back(current, position);
                std::cout << "Loaded data point " << line_count << ": current=" 
                         << current << ", position=" << position << std::endl;
            } catch (...) {
                std::cerr << "Error: Failed to parse line " << line_count 
                         << ": '" << line << "'" << std::endl;
                return false;
            }
        }
    }

    std::sort(calibration_data_.begin(), calibration_data_.end()); //lexicographical order
    std::cout << "Successfully loaded " << calibration_data_.size() 
              << " calibration points" << std::endl;
    return !calibration_data_.empty();
}

double Actuator::findNearestPosition(double current) const {
    if (calibration_data_.empty()) {
        return 0.0;
    }

    // Find the lower bound
    auto it = std::lower_bound(
        calibration_data_.begin(),
        calibration_data_.end(),
        std::make_pair(current, 0.0)
    );

    // Handle edge cases
    if (it == calibration_data_.begin()) {
        return calibration_data_.front().second;
    }
    
    if (it == calibration_data_.end()) {
        return calibration_data_.back().second;
    }

    // Find the nearest position
    auto prev = std::prev(it);
    if (std::abs(current - prev->first) < std::abs(current - it->first)) {
        return prev->second;
    }
    return it->second;
}

bool Actuator::isCurrentValid(double current) const {
    if (calibration_data_.empty()) {
        return false;
    }
    
    return current >= 0.0 && 
           current <= calibration_data_.back().first;
} 