#ifndef FORMATION_COORDINATOR_PKG__PARTICLE_SWARM_OPTIMIZER_HPP_
#define FORMATION_COORDINATOR_PKG__PARTICLE_SWARM_OPTIMIZER_HPP_

#include <vector>
#include <functional>
#include <random>
#include <limits>

namespace formation_coordinator_pkg
{

/**
 * @brief Particle Swarm Optimization (PSO) for formation optimization
 *
 * This class implements PSO algorithm to optimize formation positions
 * considering multiple objectives:
 * - Formation error minimization
 * - Collision avoidance
 * - Energy efficiency
 */
class ParticleSwarmOptimizer
{
public:
  /**
   * @brief Particle structure for PSO
   */
  struct Particle
  {
    std::vector<double> position;           // Current position
    std::vector<double> velocity;           // Current velocity
    std::vector<double> best_position;      // Personal best position
    double best_fitness;                    // Personal best fitness
    double current_fitness;                 // Current fitness value

    Particle(size_t dimensions)
    : position(dimensions, 0.0)
    , velocity(dimensions, 0.0)
    , best_position(dimensions, 0.0)
    , best_fitness(std::numeric_limits<double>::max())
    , current_fitness(std::numeric_limits<double>::max())
    {
    }
  };

  /**
   * @brief PSO Parameters structure
   */
  struct Parameters
  {
    size_t num_particles = 30;
    size_t max_iterations = 100;
    double inertia_weight = 0.9;
    double cognitive_coefficient = 2.0;     // c1
    double social_coefficient = 2.0;        // c2
    double tolerance = 1e-6;
    size_t max_stagnation_iterations = 20;
    std::vector<double> position_bounds_min;
    std::vector<double> position_bounds_max;
    std::vector<double> velocity_bounds_min;
    std::vector<double> velocity_bounds_max;
  };

  /**
   * @brief Objective function type
   * Takes a position vector and returns fitness value (lower is better)
   */
  using ObjectiveFunction = std::function<double(const std::vector<double>&)>;

  /**
   * @brief Constructor
   * @param params PSO parameters
   * @param dimensions Number of dimensions to optimize
   */
  ParticleSwarmOptimizer(const Parameters & params, size_t dimensions);

  /**
   * @brief Set the objective function to minimize
   * @param objective_func Function that evaluates fitness of a position
   */
  void setObjectiveFunction(ObjectiveFunction objective_func);

  /**
   * @brief Run the PSO optimization
   * @return Optimized position vector
   */
  std::vector<double> optimize();

  /**
   * @brief Get the best fitness value found
   * @return Best fitness value
   */
  double getBestFitness() const { return global_best_fitness_; }

  /**
   * @brief Get the current iteration number
   * @return Current iteration
   */
  size_t getCurrentIteration() const { return current_iteration_; }

  /**
   * @brief Check if optimization has converged
   * @return True if converged
   */
  bool hasConverged() const { return has_converged_; }

private:
  /**
   * @brief Initialize particle swarm with random positions and velocities
   */
  void initializeSwarm();

  /**
   * @brief Update velocities of all particles
   */
  void updateVelocities();

  /**
   * @brief Update positions of all particles
   */
  void updatePositions();

  /**
   * @brief Evaluate fitness of all particles
   */
  void evaluateFitness();

  /**
   * @brief Update personal and global best positions
   */
  void updateBests();

  /**
   * @brief Check convergence criteria
   * @return True if converged
   */
  bool checkConvergence();

  /**
   * @brief Clamp value to bounds
   */
  double clamp(double value, double min_val, double max_val) const;

  // Parameters
  Parameters params_;
  size_t dimensions_;

  // Swarm
  std::vector<Particle> particles_;
  std::vector<double> global_best_position_;
  double global_best_fitness_;

  // Optimization state
  size_t current_iteration_;
  size_t stagnation_counter_;
  bool has_converged_;

  // Objective function
  ObjectiveFunction objective_function_;

  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;
};

}  // namespace formation_coordinator_pkg

#endif  // FORMATION_COORDINATOR_PKG__PARTICLE_SWARM_OPTIMIZER_HPP_
