#ifndef AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
#define AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_

namespace agent_control_pkg
{

class PIDController
{
public:
    // Holds the breakdown of P, I, D and total output
    struct PIDTerms {
        double p;
        double i;
        double d;
        double total_output;
    };

    // Anti-windup mode selection
    enum class AntiWindupMode {
        NONE,               // No anti-windup (for testing/comparison)
        CONDITIONAL,        // Stop integration when saturated
        BACK_CALCULATION,   // Feed saturation error back to integrator
        COMBINED            // Both conditional + back-calculation (default)
    };

    // Constructor with enhanced parameters
    PIDController(double kp, double ki, double kd, double output_min, double output_max, 
                  double setpoint = 0.0, bool enable_derivative_filter = true, double derivative_filter_alpha = 0.1);

    // Update gains, output limits, or target
    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double min, double max);
    void setSetpoint(double setpoint);
    
    // Feed-forward control methods
    void enableFeedForward(bool enable, double velocity_gain = 0.8, double acceleration_gain = 0.1);
    void setFeedForwardGains(double velocity_gain, double acceleration_gain);
    
    // Derivative filter control
    void enableDerivativeFilter(bool enable, double alpha = 0.1);
    void setDerivativeFilterAlpha(double alpha);

    // Anti-windup configuration
    // Tt = tracking time constant (typical: Tt = sqrt(Ti * Td) where Ti = Kp/Ki, Td = Kd/Kp)
    void setAntiWindupMode(AntiWindupMode mode);
    void setTrackingTimeConstant(double Tt);
    AntiWindupMode getAntiWindupMode() const;

    // Access current PID settings
    double getKp() const;
    double getKi() const;
    double getKd() const;
    double getSetpoint() const;

    // Basic PID computation (returns control output)
    double calculate(double current_value, double dt);
    
    // Enhanced calculation with feed-forward
    double calculateWithFeedForward(double current_value, double dt, double setpoint_velocity = 0.0, double setpoint_acceleration = 0.0);
    
    // Version that also returns the breakdown of P/I/D
    PIDTerms calculate_with_terms(double current_value, double dt);
    


    // Retrieve last computed PID components
    PIDTerms getLastTerms() const;

    // Reset integrator and internal state
    void reset();
    
    // Enhanced diagnostic information
    double getIntegralValue() const;
    bool isOutputSaturated() const;

private:
    // PID gains
    double kp_;
    double ki_;
    double kd_;

    // Output range
    double output_min_;
    double output_max_;

    // Target value
    double setpoint_;

    // Internal memory
    double previous_error_;
    double integral_;
    bool first_calculation_;

    double previous_measurement_;  // optional, used if needed

    // Last PID terms (for logging/debug/analysis)
    PIDTerms last_terms_;
    
    // Enhanced anti-windup tracking
    bool output_saturated_;
    double last_output_before_clamp_;
    AntiWindupMode anti_windup_mode_;
    double tracking_time_constant_;  // Tt for back-calculation
    
    // Feed-forward control parameters
    bool feedforward_enabled_;
    double velocity_feedforward_gain_;
    double acceleration_feedforward_gain_;
    
    // Derivative filtering
    bool derivative_filter_enabled_;
    double derivative_filter_alpha_;
    double filtered_derivative_;
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
