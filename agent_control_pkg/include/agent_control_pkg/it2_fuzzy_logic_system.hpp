#ifndef AGENT_CONTROL_PKG__IT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__IT2_FUZZY_LOGIC_SYSTEM_HPP_

/**
 * @file it2_fuzzy_logic_system.hpp
 * @brief Interval Type-2 Fuzzy Logic System (IT2-FLS)
 *
 * This implementation uses:
 * - Interval Type-2 fuzzy sets with Footprint of Uncertainty (FOU)
 * - Triangular membership functions (lower and upper)
 * - Karnik-Mendel type reduction algorithm
 * - Centroid defuzzification
 *
 * @note This is IT2 (Interval Type-2), NOT GT2 (General Type-2).
 *       GT2 would require a full secondary membership function,
 *       whereas IT2 uses interval bounds [lower, upper].
 *
 * References:
 * - Mendel, J.M. (2001). Uncertain Rule-Based Fuzzy Logic Systems
 * - Karnik, N.N., & Mendel, J.M. (2001). Centroid of a type-2 fuzzy set
 */

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <algorithm>
#include <numeric>

// #define DEBUG_FLS // uncomment for debug prints

#ifdef DEBUG_FLS
#include <iostream>
#endif

namespace agent_control_pkg
{

/**
 * @class IT2FuzzyLogicSystem
 * @brief Interval Type-2 Fuzzy Logic System for disturbance compensation
 *
 * The IT2-FLS provides robustness to uncertainty in membership function
 * parameters through the Footprint of Uncertainty (FOU) - the bounded
 * region between lower and upper membership functions.
 */
class IT2FuzzyLogicSystem
{
public:
    /// Membership interval [lower, upper] representing IT2 uncertainty
    using MembershipInterval = std::pair<double, double>;

    /**
     * @struct IT2TriangularFS_FOU
     * @brief IT2 Triangular Fuzzy Set with Footprint of Uncertainty
     *
     * The FOU is bounded by:
     * - Lower Membership Function (LMF): inner triangle
     * - Upper Membership Function (UMF): outer triangle
     */
    struct IT2TriangularFS_FOU {
        // Lower Membership Function (LMF) parameters
        double lmf_left_base;   ///< Left support point of LMF
        double lmf_peak;        ///< Peak (mode) of LMF
        double lmf_right_base;  ///< Right support point of LMF

        // Upper Membership Function (UMF) parameters
        double umf_left_base;   ///< Left support point of UMF
        double umf_peak;        ///< Peak (mode) of UMF
        double umf_right_base;  ///< Right support point of UMF

        /**
         * @brief Get centroid of Lower Membership Function
         * @return Centroid value (currently returns peak for symmetric triangles)
         * @note For asymmetric triangles, should be (left + peak + right) / 3
         */
        double getLMFCentroid() const { return lmf_peak; }
    };

    /**
     * @struct FuzzyRule
     * @brief IF-THEN rule with multiple antecedents and single consequent
     */
    struct FuzzyRule {
        std::map<std::string, std::string> antecedents;  ///< variable -> fuzzy set
        std::pair<std::string, std::string> consequent;  ///< (variable, fuzzy set)
    };

    /**
     * @struct RuleFiringInfo
     * @brief Information about a fired rule for type reduction
     */
    struct RuleFiringInfo {
        MembershipInterval firing_strength;  ///< [f_lower, f_upper]
        double consequent_centroid;          ///< Centroid of consequent set
        size_t rule_index;                   ///< For debugging
    };

    IT2FuzzyLogicSystem();
    ~IT2FuzzyLogicSystem();

    // ========== Setup Methods ==========

    /**
     * @brief Register an input variable
     * @param var_name Variable name (e.g., "error", "dError", "wind")
     */
    void addInputVariable(const std::string& var_name);

    /**
     * @brief Register an output variable
     * @param var_name Variable name (e.g., "output", "correction")
     */
    void addOutputVariable(const std::string& var_name);

    /**
     * @brief Add a fuzzy set to a variable
     * @param var_name Variable name
     * @param set_name Set name (e.g., "NB", "ZO", "PB")
     * @param fou IT2 triangular fuzzy set with FOU
     */
    void addFuzzySetToVariable(const std::string& var_name,
                               const std::string& set_name,
                               const IT2TriangularFS_FOU& fou);

    /**
     * @brief Add a fuzzy rule
     * @param rule FuzzyRule struct with antecedents and consequent
     */
    void addRule(const FuzzyRule& rule);

    // ========== Accessors ==========

    const std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>>& getFuzzySets() const {
        return fuzzy_sets_;
    }

    /**
     * @brief Calculate triangular membership interval
     * @return [lower_membership, upper_membership]
     */
    MembershipInterval getTriangularMembership(double crisp_input,
                                               double l_left, double l_peak, double l_right,
                                               double u_left, double u_peak, double u_right);

    size_t getRuleCount() const { return rules_.size(); }

    // ========== Main Computation ==========

    /**
     * @brief Calculate IT2-FLS output
     * @param crisp_error Position error (m)
     * @param crisp_dError Error rate (m/s)
     * @param crisp_wind Wind disturbance scalar
     * @return Defuzzified crisp output (acceleration adjustment, m/s^2)
     */
    double calculateOutput(double crisp_error, double crisp_dError, double crisp_wind);

private:
    /// Fuzzy sets organized by variable name -> set name -> FOU
    std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>> fuzzy_sets_;

    /// Rule base
    std::vector<FuzzyRule> rules_;

    /// Output variable name (auto-detected from rules)
    std::string output_variable_name_;

    // ========== Internal Pipeline ==========

    /**
     * @brief Fuzzify crisp inputs to membership intervals
     */
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzifyInputs(
        double crisp_error, double crisp_dError, double crisp_wind);

    /**
     * @brief Calculate firing strengths for all rules
     */
    std::vector<RuleFiringInfo> calculateRuleFirings(
        const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);

    /**
     * @brief Karnik-Mendel type reduction algorithm
     * @param rule_firings Vector of fired rules with strengths
     * @return Type-reduced interval [y_l, y_r]
     *
     * The KM algorithm iteratively finds the switching points
     * to compute the left and right endpoints of the type-reduced set.
     */
    std::pair<double, double> typeReduce_KarnikMendel(std::vector<RuleFiringInfo>& rule_firings);

    /**
     * @brief Defuzzify type-reduced interval to crisp output
     * @param type_reduced_interval [y_l, y_r] from KM algorithm
     * @return Crisp output (average of interval endpoints)
     */
    double defuzzify(const std::pair<double, double>& type_reduced_interval);
};

// NOTE: GT2FuzzyLogicSystem is now a separate class in gt2_fuzzy_logic_system.hpp
// The old alias has been removed to avoid conflicts.

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__IT2_FUZZY_LOGIC_SYSTEM_HPP_
