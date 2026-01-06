// agent_control_pkg/src/pid_controller.cpp
#include "agent_control_pkg/pid_controller.hpp"
#include <algorithm> // For std::clamp
#include <cmath>     // For std::sqrt, std::abs
#include <iostream>  // For potential debugging

namespace agent_control_pkg
{

PIDController::PIDController(double kp, double ki, double kd, double output_min, double output_max,
                             double setpoint, bool enable_derivative_filter, double derivative_filter_alpha)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  output_min_(output_min),
  output_max_(output_max),
  setpoint_(setpoint),
  integral_(0.0),
  previous_error_(0.0),
  first_calculation_(true),
  last_terms_{0.0, 0.0, 0.0, 0.0},
  previous_measurement_(0.0),
  output_saturated_(false),
  last_output_before_clamp_(0.0),
  anti_windup_mode_(AntiWindupMode::COMBINED),  // Default: most robust
  tracking_time_constant_(0.0),  
  feedforward_enabled_(false),
  velocity_feedforward_gain_(0.8),
  acceleration_feedforward_gain_(0.1),
  derivative_filter_enabled_(enable_derivative_filter),
  derivative_filter_alpha_(derivative_filter_alpha),
  filtered_derivative_(0.0)
{
  if (output_min_ >= output_max_) {
    // This does not automatically fix the values.
    std::cerr << "PIDController Warning: Output min (" << output_min_ 
              << ") is not less than output max (" << output_max_ << ")." << std::endl;
  }
  // previous_measurement_ is initialized to 0.0. The 'first_calculation_' flag ensures
  // that the derivative term is zero on the first update, preventing derivative kick on startup.
}

void PIDController::setTunings(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double min_val, double max_val)
{
  if (min_val < max_val) {
    output_min_ = min_val;
    output_max_ = max_val;
  } else {
     std::cerr << "PIDController Warning: Invalid output limits. Min (" << min_val 
               << ") must be less than Max (" << max_val << ")." << std::endl;
  }
}

void PIDController::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
}

double PIDController::getKp() const { return kp_; }
double PIDController::getKi() const { return ki_; }
double PIDController::getKd() const { return kd_; }
double PIDController::getSetpoint() const { return setpoint_; }

PIDController::PIDTerms PIDController::calculate_with_terms(double current_value, double dt)
{
  if (dt <= 0.0) {
    last_terms_ = {0.0, 0.0, 0.0, 0.0};
    return last_terms_;
  }

  double error = setpoint_ - current_value;

  // =========================================================================
  // PROPORTIONAL TERM
  // =========================================================================
  double p_term = kp_ * error;

  // =========================================================================
  // DERIVATIVE TERM (on Measurement to avoid derivative kick)
  // D(s) = -Kd * s * Y(s)  =>  d_term = -Kd * dy/dt
  // =========================================================================
  double derivative_input_change = 0.0;
  if (first_calculation_) {
    derivative_input_change = 0.0;
    first_calculation_ = false;
  } else {
    derivative_input_change = current_value - previous_measurement_;
  }
  previous_measurement_ = current_value;

  double raw_derivative = derivative_input_change / dt;

  // Low-pass filter on derivative to reduce noise amplification
  if (derivative_filter_enabled_) {
    filtered_derivative_ = derivative_filter_alpha_ * raw_derivative +
                          (1.0 - derivative_filter_alpha_) * filtered_derivative_;
  } else {
    filtered_derivative_ = raw_derivative;
  }

  double d_term = -kd_ * filtered_derivative_;

  // =========================================================================
  // INTEGRAL TERM with Anti-Windup
  // Standard: I(k) = I(k-1) + Ki * e(k) * dt
  // Anti-windup prevents excessive integral accumulation during saturation
  // =========================================================================
  double i_term = ki_ * integral_;

  // output with current integral
  double preliminary_output = p_term + i_term + d_term;

  // conditional anti-windup
  bool should_integrate = true;

  if (anti_windup_mode_ == AntiWindupMode::CONDITIONAL ||
      anti_windup_mode_ == AntiWindupMode::COMBINED) {
    // Conditional integration: stop integrating when:
    // 1. Output is saturating high AND error is positive (would make it worse)
    // 2. Output is saturating low AND error is negative (would make it worse)
    if ((preliminary_output >= output_max_ && error > 0.0) ||
        (preliminary_output <= output_min_ && error < 0.0)) {
      should_integrate = false;
    }
  }

  if (should_integrate && anti_windup_mode_ != AntiWindupMode::NONE) {
    integral_ += error * dt;
  } else if (anti_windup_mode_ == AntiWindupMode::NONE) {
    integral_ += error * dt;
  }

  i_term = ki_ * integral_;

  previous_error_ = error;

  // =========================================================================
  // OUTPUT CALCULATION AND CLAMPING
  // =========================================================================
  double output = p_term + i_term + d_term;
  last_output_before_clamp_ = output;

  double clamped_output = std::clamp(output, output_min_, output_max_);
  output_saturated_ = (std::abs(output - clamped_output) > 1e-9);

  // =========================================================================
  // BACK-CALCULATION ANTI-WINDUP
  // es = u_saturated - u_unsaturated (saturation error)
  // dI/dt = (1/Tt) * es  =>  I += (dt/Tt) * es
  // Tt = tracking time constant (typical: Tt = sqrt(Ti*Td) or Ti)
  // =========================================================================
  if (output_saturated_ && std::abs(ki_) > 1e-9 &&
      (anti_windup_mode_ == AntiWindupMode::BACK_CALCULATION ||
       anti_windup_mode_ == AntiWindupMode::COMBINED)) {

    double saturation_error = clamped_output - output;

    // Compute tracking time constant if not explicitly set
    double Tt = tracking_time_constant_;
    if (Tt <= 0.0) {
      // Default: Tt = sqrt(Ti * Td) where Ti = Kp/Ki, Td = Kd/Kp
      // If Kd = 0, use Tt = Ti (integral time)
      double Ti = (std::abs(ki_) > 1e-9) ? kp_ / ki_ : 1.0;
      double Td = (std::abs(kp_) > 1e-9) ? kd_ / kp_ : 0.0;
      Tt = (Td > 1e-9) ? std::sqrt(Ti * Td) : Ti;
      if (Tt < dt) Tt = dt; 
    }

    double integral_correction = (dt / Tt) * saturation_error / ki_;
    integral_ += integral_correction;
  }

  // =========================================================================
  // INTEGRAL CLAMPING (safety bound)
  // =========================================================================
  if (std::abs(ki_) > 1e-9) {
    double max_integral_contribution = std::max(std::abs(output_max_), std::abs(output_min_));
    double max_integral_value = max_integral_contribution / std::abs(ki_);
    integral_ = std::clamp(integral_, -max_integral_value, max_integral_value);
  }

  last_terms_ = {p_term, i_term, d_term, clamped_output};
  return last_terms_;
}

double PIDController::calculate(double current_value, double dt)
{
  return calculate_with_terms(current_value, dt).total_output;
}

double PIDController::calculateWithFeedForward(double current_value, double dt, double setpoint_velocity, double setpoint_acceleration)
{
  // Get base PID calculation
  PIDTerms base_terms = calculate_with_terms(current_value, dt);
  
  // Add feed-forward control if enabled
  double feedforward_output = 0.0;
  if (feedforward_enabled_) {
    feedforward_output = velocity_feedforward_gain_ * setpoint_velocity + 
                        acceleration_feedforward_gain_ * setpoint_acceleration;
  }
  
  // Combine PID output with feed-forward and apply limits
  double enhanced_output = base_terms.total_output + feedforward_output;
  enhanced_output = std::clamp(enhanced_output, output_min_, output_max_);
  
  return enhanced_output;
}

PIDController::PIDTerms PIDController::getLastTerms() const
{
  return last_terms_;
}

void PIDController::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  // previous_measurement_ should ideally be reset to the current system value if known,
  // or rely on first_calculation_ to handle it.
  // Setting to 0.0 might cause a small d-term blip on the first calculation after reset if system isn't at 0.
  // A common approach is to not reset previous_measurement_ here, or set first_calculation_ to true.
  first_calculation_ = true;
  // previous_measurement_ = 0.0; // Or get current value if possible at reset time
  last_terms_ = {0.0, 0.0, 0.0, 0.0};
  output_saturated_ = false;
  last_output_before_clamp_ = 0.0;
  filtered_derivative_ = 0.0;
}

double PIDController::getIntegralValue() const
{
  return integral_;
}

bool PIDController::isOutputSaturated() const
{
  return output_saturated_;
}

void PIDController::enableFeedForward(bool enable, double velocity_gain, double acceleration_gain)
{
  feedforward_enabled_ = enable;
  velocity_feedforward_gain_ = velocity_gain;
  acceleration_feedforward_gain_ = acceleration_gain;
}

void PIDController::setFeedForwardGains(double velocity_gain, double acceleration_gain)
{
  velocity_feedforward_gain_ = velocity_gain;
  acceleration_feedforward_gain_ = acceleration_gain;
}

void PIDController::enableDerivativeFilter(bool enable, double alpha)
{
  derivative_filter_enabled_ = enable;
  derivative_filter_alpha_ = alpha;
}

void PIDController::setDerivativeFilterAlpha(double alpha)
{
  if (alpha > 0.0 && alpha <= 1.0) {
    derivative_filter_alpha_ = alpha;
  }
}

void PIDController::setAntiWindupMode(AntiWindupMode mode)
{
  anti_windup_mode_ = mode;
}

void PIDController::setTrackingTimeConstant(double Tt)
{
  tracking_time_constant_ = (Tt > 0.0) ? Tt : 0.0;
}

PIDController::AntiWindupMode PIDController::getAntiWindupMode() const
{
  return anti_windup_mode_;
}

} // namespace agent_control_pkg