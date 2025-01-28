#include <vector>
#include <string>
#include <utility>

class Actuator {
public:
    enum class State {
        Uninitialized,
        Operational,
        Error,
        Unknown
    };

    Actuator(const std::string& calibration_file);
    
    bool initialize();
    bool setCurrent(double current);
    double getPosition() const;
    State getState() const;
    bool shutdown();

private:
    std::vector<std::pair<double, double>> calibration_data_; // current, position pairs
    double current_current_;
    State current_state_;
    
    bool loadCalibrationData(const std::string& filename);
    double findNearestPosition(double current) const;
    bool isCurrentValid(double current) const;
}; 