#include "formation_coordinator_pkg/particle_swarm_optimizer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace formation_coordinator_pkg
{

ParticleSwarmOptimizer::ParticleSwarmOptimizer(
  const Parameters & params,
  size_t dimensions)
: params_(params)
, dimensions_(dimensions)
, global_best_fitness_(std::numeric_limits<double>::max())
, current_iteration_(0)
, stagnation_counter_(0)
, has_converged_(false)
, uniform_dist_(0.0, 1.0)
{
  // Initialize random number generator with time-based seed
  rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());

  // Initialize global best position
  global_best_position_.resize(dimensions_, 0.0);

  // Validate and set default bounds if not provided
  if (params_.position_bounds_min.empty()) {
    params_.position_bounds_min.assign(dimensions_, -10.0);
  }
  if (params_.position_bounds_max.empty()) {
    params_.position_bounds_max.assign(dimensions_, 10.0);
  }
  if (params_.velocity_bounds_min.empty()) {
    params_.velocity_bounds_min.assign(dimensions_, -2.0);
  }
  if (params_.velocity_bounds_max.empty()) {
    params_.velocity_bounds_max.assign(dimensions_, 2.0);
  }

  // Initialize swarm
  initializeSwarm();
}

void ParticleSwarmOptimizer::setObjectiveFunction(ObjectiveFunction objective_func)
{
  objective_function_ = objective_func;
}

void ParticleSwarmOptimizer::initializeSwarm()
{
  particles_.clear();
  particles_.reserve(params_.num_particles);

  for (size_t i = 0; i < params_.num_particles; ++i) {
    Particle particle(dimensions_);

    // Initialize position within bounds
    for (size_t d = 0; d < dimensions_; ++d) {
      double range = params_.position_bounds_max[d] - params_.position_bounds_min[d];
      particle.position[d] = params_.position_bounds_min[d] + uniform_dist_(rng_) * range;
      particle.best_position[d] = particle.position[d];

      // Initialize velocity
      double vel_range = params_.velocity_bounds_max[d] - params_.velocity_bounds_min[d];
      particle.velocity[d] = params_.velocity_bounds_min[d] + uniform_dist_(rng_) * vel_range;
    }

    particles_.push_back(particle);
  }
}

std::vector<double> ParticleSwarmOptimizer::optimize()
{
  if (!objective_function_) {
    throw std::runtime_error("Objective function not set!");
  }

  // Initial fitness evaluation
  evaluateFitness();
  updateBests();

  // Main PSO loop
  for (current_iteration_ = 0;
    current_iteration_ < params_.max_iterations;
    ++current_iteration_)
  {
    // Update particle velocities and positions
    updateVelocities();
    updatePositions();

    // Evaluate fitness
    evaluateFitness();

    // Update personal and global bests
    updateBests();

    // Check convergence
    if (checkConvergence()) {
      has_converged_ = true;
      break;
    }
  }

  return global_best_position_;
}

void ParticleSwarmOptimizer::updateVelocities()
{
  for (auto & particle : particles_) {
    for (size_t d = 0; d < dimensions_; ++d) {
      // Random factors for stochastic behavior
      double r1 = uniform_dist_(rng_);
      double r2 = uniform_dist_(rng_);

      // PSO velocity update equation
      double cognitive_component =
        params_.cognitive_coefficient * r1 *
        (particle.best_position[d] - particle.position[d]);

      double social_component =
        params_.social_coefficient * r2 *
        (global_best_position_[d] - particle.position[d]);

      particle.velocity[d] =
        params_.inertia_weight * particle.velocity[d] +
        cognitive_component +
        social_component;

      // Clamp velocity to bounds
      particle.velocity[d] = clamp(
        particle.velocity[d],
        params_.velocity_bounds_min[d],
        params_.velocity_bounds_max[d]);
    }
  }
}

void ParticleSwarmOptimizer::updatePositions()
{
  for (auto & particle : particles_) {
    for (size_t d = 0; d < dimensions_; ++d) {
      // Update position
      particle.position[d] += particle.velocity[d];

      // Clamp position to bounds
      particle.position[d] = clamp(
        particle.position[d],
        params_.position_bounds_min[d],
        params_.position_bounds_max[d]);
    }
  }
}

void ParticleSwarmOptimizer::evaluateFitness()
{
  for (auto & particle : particles_) {
    particle.current_fitness = objective_function_(particle.position);
  }
}

void ParticleSwarmOptimizer::updateBests()
{
  double previous_global_best = global_best_fitness_;

  for (auto & particle : particles_) {
    // Update personal best
    if (particle.current_fitness < particle.best_fitness) {
      particle.best_fitness = particle.current_fitness;
      particle.best_position = particle.position;
    }

    // Update global best
    if (particle.best_fitness < global_best_fitness_) {
      global_best_fitness_ = particle.best_fitness;
      global_best_position_ = particle.best_position;
    }
  }

  // Check for stagnation
  if (std::abs(global_best_fitness_ - previous_global_best) < params_.tolerance) {
    ++stagnation_counter_;
  } else {
    stagnation_counter_ = 0;
  }
}

bool ParticleSwarmOptimizer::checkConvergence()
{
  // Check if fitness is good enough
  if (global_best_fitness_ < params_.tolerance) {
    return true;
  }

  // Check for stagnation
  if (stagnation_counter_ >= params_.max_stagnation_iterations) {
    return true;
  }

  // Check if maximum iterations reached
  if (current_iteration_ >= params_.max_iterations - 1) {
    return true;
  }

  return false;
}

double ParticleSwarmOptimizer::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(value, max_val));
}

}  // namespace formation_coordinator_pkg
