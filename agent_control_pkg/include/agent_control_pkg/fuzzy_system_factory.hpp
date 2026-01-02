#ifndef AGENT_CONTROL_PKG__FUZZY_SYSTEM_FACTORY_HPP_
#define AGENT_CONTROL_PKG__FUZZY_SYSTEM_FACTORY_HPP_

/**
 * @file fuzzy_system_factory.hpp
 * @brief Factory and adapter for IT2 and GT2 fuzzy logic systems
 *
 * Provides unified interface for both fuzzy system types,
 * enabling easy comparison and configuration switching.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include "agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <memory>
#include <string>
#include <variant>

namespace agent_control_pkg
{

/**
 * @enum FuzzySystemType
 * @brief Available fuzzy logic system types
 */
enum class FuzzySystemType {
    IT2,    ///< Interval Type-2 (simpler, faster)
    GT2     ///< General Type-2 (more accurate, slower)
};

/**
 * @class FuzzySystemInterface
 * @brief Abstract interface for fuzzy systems
 */
class FuzzySystemInterface
{
public:
    virtual ~FuzzySystemInterface() = default;

    virtual double calculateOutput(double error, double dError, double wind) = 0;
    virtual std::string getTypeName() const = 0;
    virtual size_t getRuleCount() const = 0;
};

/**
 * @class IT2FuzzyAdapter
 * @brief Adapter wrapping IT2FuzzyLogicSystem
 */
class IT2FuzzyAdapter : public FuzzySystemInterface
{
public:
    explicit IT2FuzzyAdapter(std::shared_ptr<IT2FuzzyLogicSystem> system)
        : system_(std::move(system)) {}

    double calculateOutput(double error, double dError, double wind) override {
        return system_->calculateOutput(error, dError, wind);
    }

    std::string getTypeName() const override { return "IT2-FLS"; }
    size_t getRuleCount() const override { return system_->getRuleCount(); }

    IT2FuzzyLogicSystem* getSystem() { return system_.get(); }

private:
    std::shared_ptr<IT2FuzzyLogicSystem> system_;
};

/**
 * @class GT2FuzzyAdapter
 * @brief Adapter wrapping GT2FuzzyLogicSystem
 */
class GT2FuzzyAdapter : public FuzzySystemInterface
{
public:
    explicit GT2FuzzyAdapter(std::shared_ptr<GT2FuzzyLogicSystem> system)
        : system_(std::move(system)) {}

    double calculateOutput(double error, double dError, double wind) override {
        return system_->calculateOutput(error, dError, wind);
    }

    std::string getTypeName() const override { return "GT2-FLS"; }
    size_t getRuleCount() const override { return system_->getRuleCount(); }

    GT2FuzzyLogicSystem* getSystem() { return system_.get(); }

    const std::vector<GT2FuzzyLogicSystem::AlphaPlaneResult>& getLastAlphaResults() const {
        return system_->getLastAlphaPlaneResults();
    }

private:
    std::shared_ptr<GT2FuzzyLogicSystem> system_;
};

/**
 * @class FuzzySystemFactory
 * @brief Factory for creating and configuring fuzzy systems
 */
class FuzzySystemFactory
{
public:
    /**
     * @brief Create a fuzzy system with standard rule base
     * @param type IT2 or GT2
     * @param num_alpha_levels For GT2: number of alpha planes (default: 5)
     * @param secondary_shape For GT2: shape of secondary MF
     * @return Configured fuzzy system interface
     */
    static std::unique_ptr<FuzzySystemInterface> create(
        FuzzySystemType type,
        int num_alpha_levels = 5,
        GT2FuzzyLogicSystem::SecondaryMFShape secondary_shape =
            GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);

    /**
     * @brief Create IT2 system with standard configuration
     */
    static std::unique_ptr<IT2FuzzyAdapter> createIT2();

    /**
     * @brief Create GT2 system with specified configuration
     */
    static std::unique_ptr<GT2FuzzyAdapter> createGT2(
        int num_alpha_levels = 5,
        GT2FuzzyLogicSystem::SecondaryMFShape secondary_shape =
            GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);

private:
    /**
     * @brief Add standard fuzzy sets to IT2 system
     */
    static void configureIT2Sets(IT2FuzzyLogicSystem* system);

    /**
     * @brief Add standard fuzzy sets to GT2 system
     */
    static void configureGT2Sets(GT2FuzzyLogicSystem* system,
                                 GT2FuzzyLogicSystem::SecondaryMFShape shape);

    /**
     * @brief Add standard rule base (same for both systems)
     */
    template<typename SystemType>
    static void addStandardRules(SystemType* system);
};

// ========== String Conversion Utilities ==========

inline std::string fuzzyTypeToString(FuzzySystemType type) {
    switch (type) {
        case FuzzySystemType::IT2: return "IT2";
        case FuzzySystemType::GT2: return "GT2";
        default: return "Unknown";
    }
}

inline FuzzySystemType stringToFuzzyType(const std::string& str) {
    if (str == "GT2" || str == "gt2" || str == "general") {
        return FuzzySystemType::GT2;
    }
    return FuzzySystemType::IT2;  // Default
}

inline std::string secondaryShapeToString(GT2FuzzyLogicSystem::SecondaryMFShape shape) {
    switch (shape) {
        case GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR: return "Triangular";
        case GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN: return "Gaussian";
        case GT2FuzzyLogicSystem::SecondaryMFShape::TRAPEZOIDAL: return "Trapezoidal";
        case GT2FuzzyLogicSystem::SecondaryMFShape::UNIFORM: return "Uniform (IT2-like)";
        default: return "Unknown";
    }
}

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__FUZZY_SYSTEM_FACTORY_HPP_
