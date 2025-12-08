# Interval Type-2 Fuzzy Logic System (IT2-FLS)

This document describes the Interval Type-2 Fuzzy Logic System implementation used for disturbance compensation in the multi-agent formation control system.

## 1. Type-2 Fuzzy Sets Overview

### 1.1 Motivation

Standard Type-1 fuzzy sets use crisp membership functions, which cannot capture uncertainty in the membership grades themselves. Type-2 fuzzy sets address this by having a **Footprint of Uncertainty (FOU)** bounded by upper and lower membership functions.

### 1.2 Type-2 vs Type-1

| Aspect | Type-1 | Interval Type-2 |
|--------|--------|-----------------|
| Membership function | Single curve | FOU (upper + lower bounds) |
| Uncertainty modeling | None | In membership grades |
| Computational cost | Low | Medium |
| Robustness to noise | Baseline | Improved |

### 1.3 Why Interval Type-2 (IT2)?

- **IT2** uses an interval (uniform) secondary membership function
- Simpler than General Type-2 (GT2) which has arbitrary secondary MFs
- Well-established type-reduction algorithms (Karnik-Mendel)
- Good balance between modeling power and computational efficiency

## 2. Mathematical Formulation

### 2.1 IT2 Fuzzy Set

An IT2 fuzzy set $\tilde{A}$ is characterized by its upper and lower membership functions:

$$
\tilde{A} = \{(x, [\underline{\mu}_{\tilde{A}}(x), \overline{\mu}_{\tilde{A}}(x)]) | x \in X\}
$$

where:
- $\overline{\mu}_{\tilde{A}}(x)$ : Upper Membership Function (UMF)
- $\underline{\mu}_{\tilde{A}}(x)$ : Lower Membership Function (LMF)
- $0 \leq \underline{\mu}(x) \leq \overline{\mu}(x) \leq 1$

### 2.2 Triangular IT2 Fuzzy Set

For a triangular IT2 set with parameters:
- Upper: $(a_U, b_U, c_U)$
- Lower: $(a_L, b_L, c_L)$

$$
\overline{\mu}(x) = \begin{cases}
0 & x < a_U \\
\frac{x - a_U}{b_U - a_U} & a_U \leq x < b_U \\
\frac{c_U - x}{c_U - b_U} & b_U \leq x < c_U \\
0 & x \geq c_U
\end{cases}
$$

$$
\underline{\mu}(x) = \begin{cases}
0 & x < a_L \\
\frac{x - a_L}{b_L - a_L} & a_L \leq x < b_L \\
\frac{c_L - x}{c_L - b_L} & b_L \leq x < c_L \\
0 & x \geq c_L
\end{cases}
$$

The FOU is the area between these two curves.

## 3. IT2-FLS Structure

### 3.1 Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          IT2-FLS PIPELINE                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Crisp        ┌──────────┐     ┌──────────┐     ┌──────────────┐          │
│   Inputs  ───▶ │ Fuzzify  │ ──▶ │ Inference│ ──▶ │Type Reduction│          │
│  (e, de/dt)    │ (IT2 MFs)│     │ (Rules)  │     │   (K-M)      │          │
│                └──────────┘     └──────────┘     └──────┬───────┘          │
│                                                          │                   │
│                                                          ▼                   │
│                                                   ┌──────────────┐          │
│                                                   │  Defuzzify   │          │
│                                                   │  (Average)   │          │
│                                                   └──────┬───────┘          │
│                                                          │                   │
│                                                          ▼                   │
│                                                    Crisp Output              │
│                                                    (correction)              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Inference Engine

For each rule $R^l$: IF $x_1$ is $\tilde{A}_1^l$ AND $x_2$ is $\tilde{A}_2^l$ THEN $y$ is $\tilde{B}^l$

**Firing intervals:**
$$
[\underline{f}^l, \overline{f}^l] = [\prod_i \underline{\mu}_{\tilde{A}_i^l}(x_i), \prod_i \overline{\mu}_{\tilde{A}_i^l}(x_i)]
$$

Using product (min) t-norm for AND operation.

## 4. Karnik-Mendel Type Reduction

### 4.1 Algorithm Overview

The K-M algorithm computes the centroid of the type-reduced set:

$$
[y_l, y_r] = \left[\frac{\sum_{i=1}^{L} \underline{f}^i \cdot y^i + \sum_{i=L+1}^{N} \overline{f}^i \cdot y^i}{\sum_{i=1}^{L} \underline{f}^i + \sum_{i=L+1}^{N} \overline{f}^i}, \frac{\sum_{i=1}^{R} \overline{f}^i \cdot y^i + \sum_{i=R+1}^{N} \underline{f}^i \cdot y^i}{\sum_{i=1}^{R} \overline{f}^i + \sum_{i=R+1}^{N} \underline{f}^i}\right]
$$

where $L$ and $R$ are switch points found iteratively.

### 4.2 K-M Algorithm Steps (for $y_l$)

1. Sort rules by consequent centroid $y^i$ in ascending order
2. Initialize: $L = N/2$, compute initial $y_l$
3. Find switch point $L$ where $y^L \leq y_l \leq y^{L+1}$
4. Iterate until convergence (typically 2-3 iterations)

### 4.3 Defuzzification

Final crisp output:
$$
y = \frac{y_l + y_r}{2}
$$

## 5. Rule Base Design

### 5.1 Input Variables

| Variable | Symbol | Range | Physical Meaning |
|----------|--------|-------|------------------|
| Error | $e$ | [-5, 5] m | Position error |
| Error Rate | $\dot{e}$ | [-2, 2] m/s | Velocity error |
| Wind (optional) | $w$ | [0, 5] m/s | Wind magnitude |

### 5.2 Linguistic Terms (IEEE Standard)

- **NB**: Negative Big
- **NS**: Negative Small
- **ZO**: Zero
- **PS**: Positive Small
- **PB**: Positive Big

### 5.3 Rule Matrix (21 Rules)

Standard Mamdani-style rules for error compensation:

| e \ de | NB | NS | ZO | PS | PB |
|--------|-----|-----|-----|-----|-----|
| NB | NB | NB | NB | NS | ZO |
| NS | NB | NS | NS | ZO | PS |
| ZO | NS | NS | ZO | PS | PS |
| PS | NS | ZO | PS | PS | PB |
| PB | ZO | PS | PB | PB | PB |

Note: Some combinations may be omitted for sparse rule base (21 instead of 25 rules).

### 5.4 Output Scaling

Output membership function peaks are scaled to match acceleration units:
- NB: -6.0 m/s²
- NS: -3.0 m/s²
- ZO: 0.0 m/s²
- PS: +3.0 m/s²
- PB: +6.0 m/s²

## 6. Implementation Details

### 6.1 Class Structure

```cpp
class IT2FuzzyLogicSystem {
public:
    // Input/output variable management
    void addInputVariable(const std::string& name);
    void addOutputVariable(const std::string& name);

    // Fuzzy set definition (triangular FOU)
    void addFuzzySetToVariable(const std::string& var,
                               const std::string& set_name,
                               const IT2TriangularFS_FOU& fou);

    // Rule addition
    void addRule(const FuzzyRule& rule);

    // Main inference
    double evaluate(const std::unordered_map<std::string, double>& inputs);

private:
    // Karnik-Mendel type reduction
    std::pair<double, double> typeReduceKM(
        const std::vector<RuleFiring>& firings) const;
};
```

### 6.2 Configuration (YAML)

```yaml
fuzzy:
  enable: true
  include_wind: false
  wind_scalar: 0.0
  params_file: fuzzy_params.yaml
```

### 6.3 Hybrid Controller Integration

In PID+Fuzzy hybrid mode:

$$
u_{total} = k_{pid} \cdot u_{pid} + k_{fuzzy} \cdot u_{fuzzy}
$$

where $k_{pid}$ and $k_{fuzzy}$ are mixing gains (typically both = 1.0).

## 7. Advantages of IT2-FLS for Formation Control

1. **Robustness to Uncertainty:** FOU captures uncertainty in membership boundaries
2. **Smooth Output:** Type reduction provides naturally smooth control signals
3. **Noise Rejection:** Interval membership grades are less sensitive to input noise
4. **Adaptive Behavior:** Rules can encode expert knowledge about disturbance patterns

## 8. Comparison: IT2 vs GT2

| Aspect | IT2 (Implemented) | GT2 (Future Work) |
|--------|-------------------|-------------------|
| Secondary MF | Uniform (interval) | Arbitrary shape |
| Computation | O(N log N) for K-M | O(N × α-levels) |
| Expressiveness | Medium | High |
| Thesis scope | Primary contribution | Extension |

## 9. References

1. Mendel, J.M. (2001). *Uncertain Rule-Based Fuzzy Logic Systems: Introduction and New Directions*. Prentice Hall.
2. Karnik, N.N., & Mendel, J.M. (1998). "Introduction to Type-2 Fuzzy Logic Systems". *IEEE International Conference on Fuzzy Systems*.
3. Wu, D. (2012). "On the Fundamental Differences Between Interval Type-2 and Type-1 Fuzzy Logic Controllers". *IEEE Transactions on Fuzzy Systems*.
4. Castillo, O., & Melin, P. (2008). *Type-2 Fuzzy Logic: Theory and Applications*. Springer.

---

**Document Version:** 1.0
**Last Updated:** 2024-12-09
**Author:** Control Systems Research Team
