#ifndef AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_

/**
 * @file fuzzy_gt2_adapter.hpp
 * @brief Adapter for GT2 (General Type-2) Fuzzy Logic System as IController1D
 *
 * Bridges the GT2FuzzyLogicSystem to the controller interface,
 * handling error/dError calculation and output saturation.
 *
 * GT2 uses alpha-plane decomposition for full secondary membership,
 * providing more nuanced handling of uncertainty compared to IT2.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include "agent_control_pkg/controllers/controller_base.hpp"
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <memory>
#include <string>
#include <map>
#include <array>
#include <vector>

namespace agent_control_pkg::controllers {

/**
 * @class FuzzyGT2Adapter
 * @brief Controller adapter for General Type-2 Fuzzy Logic System
 *
 * Similar interface to FuzzyIT2Adapter but uses GT2FuzzyLogicSystem
 * with alpha-plane computation for handling secondary membership.
 */
class FuzzyGT2Adapter : public IController1D {
public:
    struct Options {
        double umin{-12.0};         ///< Output lower limit
        double umax{12.0};          ///< Output upper limit
        double wind_scalar{0.0};    ///< Wind disturbance scalar for fuzzy input
        int num_alpha_levels{5};    ///< Number of alpha-planes for GT2
        GT2FuzzyLogicSystem::SecondaryMFShape secondary_shape{
            GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR
        };  ///< Secondary MF shape
    };

    explicit FuzzyGT2Adapter(const Options &opt);

    /**
     * @brief Configure fuzzy system from parsed parameters
     * @param sets Map of variable name -> set name -> FOU parameters
     * @param rules Vector of rules: [error_label, dError_label, wind_label, output_label]
     * @param include_wind Whether to include wind as fuzzy input
     */
    void configureFromParams(
        const std::map<std::string,
                       std::map<std::string, GT2FuzzyLogicSystem::GT2TriangularFS>> &sets,
        const std::vector<std::array<std::string, 4>> &rules,
        bool include_wind);

    /**
     * @brief Configure with default rule base (using factory)
     * @param include_wind Whether to include wind input
     */
    void configureDefault(bool include_wind = true);

    /**
     * @brief Compute control output
     * @param y Current measurement
     * @param yref Reference setpoint
     * @param dt Time step (s)
     * @return Control command (saturated)
     */
    double compute(double y, double yref, double dt) override;

    void reset() override;

    void setWindScalar(double w) { opt_.wind_scalar = w; }
    void setLimits(double umin, double umax) { opt_.umin = umin; opt_.umax = umax; }
    void setNumAlphaLevels(int n);
    void setSecondaryShape(GT2FuzzyLogicSystem::SecondaryMFShape shape);

    /// Get reference to internal GT2-FLS for inspection
    const GT2FuzzyLogicSystem& fls() const { return fls_; }

    /// Get last alpha-plane results (for debugging/analysis)
    const std::vector<GT2FuzzyLogicSystem::AlphaPlaneResult>& getLastAlphaResults() const {
        return fls_.getLastAlphaPlaneResults();
    }

    /// Get type name for identification
    std::string getTypeName() const { return "GT2-FLS"; }

private:
    Options opt_;
    GT2FuzzyLogicSystem fls_;
    bool include_wind_{false};
    bool initialized_{false};
    double prev_error_{0.0};
    bool first_{true};
};

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_
