#ifndef AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_

/**
 * @file gt2_fuzzy_logic_system.hpp
 * @brief General Type-2 Fuzzy Logic System (GT2-FLS) using Alpha-Plane Method
 *
 * This implementation uses the alpha-plane (zSlices) approach:
 * - Discretizes secondary membership function into α-cuts
 * - Each α-cut produces an IT2 set
 * - Combines results using weighted average across alpha levels
 *
 * Key Differences from IT2:
 * - IT2: Secondary MF is constant 1 over interval [lower, upper]
 * - GT2: Full secondary MF μ̃(x,u) where u ∈ [0,1]
 *
 * Mathematical Foundation:
 * GT2 fuzzy set: Ã = ∫∫ μ_Ã(x,u)/u, u ∈ [0,1], x ∈ X
 *
 * Alpha-Plane Method:
 * For α ∈ {α₁, α₂, ..., αₙ}:
 *   Ã_α = IT2 set at alpha level α
 *   y = Σ(αᵢ · yᵢ) / Σ(αᵢ)  where yᵢ = IT2 output at αᵢ
 *
 * References:
 * - Liu, F. (2008). "An efficient centroid type-reduction strategy for GT2 FLS"
 * - Wagner, C., & Hagras, H. (2010). "zSlices based GT2 FLS"
 * - Mendel, J.M. (2017). "Uncertain Rule-Based Fuzzy Systems" (2nd Ed.)
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <functional>
#include <cmath>

namespace agent_control_pkg
{

/**
 * @class GT2FuzzyLogicSystem
 * @brief General Type-2 Fuzzy Logic System with full secondary membership
 *
 * Uses alpha-plane decomposition to handle the full 3D fuzzy set.
 * More computationally expensive than IT2 but captures more uncertainty.
 */
class GT2FuzzyLogicSystem
{
public:
    // ========== Type Definitions ==========

    /// Membership interval for IT2 at specific alpha level
    using MembershipInterval = std::pair<double, double>;

    /// Secondary membership function type: μ(u) for u ∈ [0,1]
    /// Returns the secondary grade at primary membership level u
    using SecondaryMFType = std::function<double(double)>;

    /**
     * @enum SecondaryMFShape
     * @brief Predefined secondary membership function shapes
     */
    enum class SecondaryMFShape {
        TRIANGULAR,     ///< Triangle peak at center of FOU
        GAUSSIAN,       ///< Gaussian centered at FOU midpoint
        TRAPEZOIDAL,    ///< Trapezoid across FOU
        UNIFORM         ///< Constant 1 (reduces to IT2)
    };

    /**
     * @struct GT2TriangularFS
     * @brief GT2 Triangular Fuzzy Set with Full Secondary MF
     *
     * Primary MF: Triangular with FOU (like IT2)
     * Secondary MF: Customizable function over the FOU interval
     */
    struct GT2TriangularFS {
        // Primary MF bounds (same as IT2)
        double lmf_left_base;   ///< Left support of LMF
        double lmf_peak;        ///< Peak of LMF
        double lmf_right_base;  ///< Right support of LMF

        double umf_left_base;   ///< Left support of UMF
        double umf_peak;        ///< Peak of UMF
        double umf_right_base;  ///< Right support of UMF

        // Secondary MF parameters
        SecondaryMFShape secondary_shape;  ///< Shape of secondary MF
        double secondary_spread;           ///< Spread parameter (for Gaussian)
        double secondary_peak_location;    ///< Peak location in [0,1] relative to FOU

        /**
         * @brief Get centroid of this fuzzy set
         */
        double getCentroid() const { return lmf_peak; }

        /**
         * @brief Calculate secondary membership at u given FOU bounds
         * @param u Primary membership level ∈ [lower, upper]
         * @param lower Lower bound from LMF
         * @param upper Upper bound from UMF
         * @return Secondary membership μ(u)
         */
        double getSecondaryMembership(double u, double lower, double upper) const;
    };

    /**
     * @struct FuzzyRule
     * @brief IF-THEN fuzzy rule
     */
    struct FuzzyRule {
        std::map<std::string, std::string> antecedents;  ///< var -> set
        std::pair<std::string, std::string> consequent;  ///< (var, set)
    };

    /**
     * @struct AlphaPlaneResult
     * @brief Result from processing one alpha-plane
     */
    struct AlphaPlaneResult {
        double alpha_level;     ///< The α value [0,1]
        double y_left;          ///< Left endpoint of type-reduced interval
        double y_right;         ///< Right endpoint of type-reduced interval
        double y_centroid;      ///< Centroid (y_left + y_right) / 2
        double weight;          ///< Weight for final combination
    };

    // ========== Configuration ==========

    /**
     * @struct GT2Config
     * @brief Configuration parameters for GT2-FLS
     */
    struct GT2Config {
        int num_alpha_levels;           ///< Number of α-planes (default: 5)
        bool use_uniform_alpha;         ///< Use uniform α spacing (default: true)
        std::vector<double> alpha_values; ///< Custom α values if not uniform
        SecondaryMFShape default_secondary_shape; ///< Default secondary MF shape
        double secondary_spread;        ///< Default spread for Gaussian secondary MF

        GT2Config()
            : num_alpha_levels(5)
            , use_uniform_alpha(true)
            , default_secondary_shape(SecondaryMFShape::TRIANGULAR)
            , secondary_spread(0.3)
        {}
    };

    // ========== Constructor/Destructor ==========

    GT2FuzzyLogicSystem();
    explicit GT2FuzzyLogicSystem(const GT2Config& config);
    ~GT2FuzzyLogicSystem();

    // ========== Setup Methods ==========

    void setConfig(const GT2Config& config);
    void addInputVariable(const std::string& var_name);
    void addOutputVariable(const std::string& var_name);
    void addFuzzySetToVariable(const std::string& var_name,
                               const std::string& set_name,
                               const GT2TriangularFS& fs);
    void addRule(const FuzzyRule& rule);

    // ========== Accessors ==========

    const GT2Config& getConfig() const { return config_; }
    size_t getRuleCount() const { return rules_.size(); }
    int getNumAlphaLevels() const { return config_.num_alpha_levels; }

    /**
     * @brief Get the α values used for alpha-plane decomposition
     */
    std::vector<double> getAlphaLevels() const;

    // ========== Main Computation ==========

    /**
     * @brief Calculate GT2-FLS output using alpha-plane method
     * @param crisp_error Position error (m)
     * @param crisp_dError Error rate (m/s)
     * @param crisp_wind Wind disturbance scalar
     * @return Defuzzified crisp output
     */
    double calculateOutput(double crisp_error, double crisp_dError, double crisp_wind);

    /**
     * @brief Get detailed alpha-plane results from last computation
     */
    const std::vector<AlphaPlaneResult>& getLastAlphaPlaneResults() const {
        return last_alpha_results_;
    }

    /**
     * @brief Get type-reduced interval from last computation
     */
    std::pair<double, double> getLastTypeReducedInterval() const {
        return last_type_reduced_interval_;
    }

private:
    GT2Config config_;

    /// Fuzzy sets: variable -> set name -> GT2 fuzzy set
    std::map<std::string, std::map<std::string, GT2TriangularFS>> fuzzy_sets_;

    /// Rule base
    std::vector<FuzzyRule> rules_;

    /// Output variable name
    std::string output_variable_name_;

    /// Cache for last computation results
    std::vector<AlphaPlaneResult> last_alpha_results_;
    std::pair<double, double> last_type_reduced_interval_;

    // ========== Internal Pipeline ==========

    /**
     * @brief Generate α-levels based on configuration
     */
    std::vector<double> generateAlphaLevels() const;

    /**
     * @brief Get membership interval at specific α-level
     *
     * For GT2, the FOU shrinks as α increases:
     * At α=0: Full FOU [lower, upper]
     * At α=1: Single point (peak of secondary MF)
     */
    MembershipInterval getAlphaCutMembership(
        double crisp_input,
        const GT2TriangularFS& fs,
        double alpha_level);

    /**
     * @brief Process single α-plane (essentially IT2 computation)
     */
    AlphaPlaneResult processAlphaPlane(
        double alpha_level,
        double crisp_error,
        double crisp_dError,
        double crisp_wind);

    /**
     * @brief Fuzzify inputs at specific α-level
     */
    std::map<std::string, std::map<std::string, MembershipInterval>>
    fuzzifyInputsAtAlpha(double alpha_level,
                         double crisp_error,
                         double crisp_dError,
                         double crisp_wind);

    /**
     * @brief Calculate rule firings (same as IT2)
     */
    struct RuleFiringInfo {
        MembershipInterval firing_strength;
        double consequent_centroid;
        size_t rule_index;
    };

    std::vector<RuleFiringInfo> calculateRuleFirings(
        const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);

    /**
     * @brief Karnik-Mendel type reduction (reused from IT2)
     */
    std::pair<double, double> typeReduce_KarnikMendel(
        std::vector<RuleFiringInfo>& rule_firings);

    /**
     * @brief Combine α-plane results using weighted average
     *
     * Final output = Σ(weight_i × centroid_i) / Σ(weight_i)
     * where weight_i = α_i (or custom weighting)
     */
    double combineAlphaPlaneResults(const std::vector<AlphaPlaneResult>& results);

    /**
     * @brief Calculate secondary membership weight at α-level
     */
    double calculateSecondaryWeight(double alpha_level) const;
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
