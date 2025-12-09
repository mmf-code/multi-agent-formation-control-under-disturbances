#!/usr/bin/env python3
"""
ANOVA Statistical Analysis for Controller Comparison

Performs one-way ANOVA with Tukey HSD post-hoc analysis to validate
performance differences between PID+IT2-FLS, PD, and PID controllers.

Usage:
    python3 scripts/anova_statistical_analysis.py thesis_data/anova/

Requirements:
    - Multiple run directories (run_1, run_2, ..., run_N)
    - Each run contains CSV files for agent_0, agent_3, agent_6

Outputs:
    - Console summary with ANOVA results
    - CSV file with all statistics
    - Box plot visualization
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from typing import Dict, List, Tuple
from dataclasses import dataclass
import warnings

# Statistical imports
from scipy import stats
try:
    from statsmodels.stats.multicomp import pairwise_tukeyhsd
    HAS_STATSMODELS = True
except ImportError:
    HAS_STATSMODELS = False
    warnings.warn("statsmodels not installed. Tukey HSD will be skipped.")


@dataclass
class ANOVAResults:
    """Container for ANOVA analysis results"""
    metric_name: str
    n_per_group: int
    group_means: Dict[str, float]
    group_stds: Dict[str, float]
    f_statistic: float
    p_value: float
    eta_squared: float
    tukey_results: str
    cohens_d: Dict[str, float]


class ANOVAAnalyzer:
    """Perform ANOVA analysis on controller comparison data"""

    CONTROLLERS = {
        'agent_0': 'PID+Fuzzy',
        'agent_3': 'PD',
        'agent_6': 'PID'
    }

    COLORS = {
        'PID+Fuzzy': '#2E7D32',
        'PD': '#1976D2',
        'PID': '#D32F2F'
    }

    def __init__(self, base_dir: str):
        self.base_dir = Path(base_dir)
        self.data: Dict[str, List[float]] = {name: [] for name in self.CONTROLLERS.values()}
        self.raw_data: Dict[str, List[pd.DataFrame]] = {name: [] for name in self.CONTROLLERS.values()}

    def find_run_directories(self) -> List[Path]:
        """Find all run_X directories"""
        run_dirs = sorted(self.base_dir.glob("run_*"))
        if not run_dirs:
            # Try direct directory if no run_X subdirectories
            run_dirs = [self.base_dir]
        return run_dirs

    def compute_rmse_from_df(self, df: pd.DataFrame) -> float:
        """Compute RMSE from a dataframe with error columns"""
        # Skip first 5 seconds (settling time)
        mask = df['elapsed_time'] > 5.0
        error_x = df.loc[mask, 'error_x'].values
        error_y = df.loc[mask, 'error_y'].values

        if len(error_x) == 0:
            return np.nan

        error_total = np.sqrt(error_x**2 + error_y**2)
        rmse = np.sqrt(np.mean(error_total**2))
        return rmse

    def load_all_runs(self) -> int:
        """Load data from all run directories"""
        run_dirs = self.find_run_directories()

        print("\n" + "=" * 70)
        print("  ANOVA STATISTICAL ANALYSIS")
        print("=" * 70)
        print(f"\nBase directory: {self.base_dir}")
        print(f"Found {len(run_dirs)} run directories\n")

        for run_dir in run_dirs:
            print(f"Loading: {run_dir.name}")

            for agent_id, controller_name in self.CONTROLLERS.items():
                csv_files = list(run_dir.glob(f"**/{agent_id}_*.csv"))

                if not csv_files:
                    print(f"  WARNING: No data for {agent_id} in {run_dir.name}")
                    continue

                csv_path = csv_files[0]
                df = pd.read_csv(csv_path)
                rmse = self.compute_rmse_from_df(df)

                if not np.isnan(rmse):
                    self.data[controller_name].append(rmse)
                    self.raw_data[controller_name].append(df)

        # Verify equal sample sizes
        sizes = {name: len(values) for name, values in self.data.items()}
        print(f"\nSample sizes: {sizes}")

        return min(sizes.values()) if sizes else 0

    def run_anova(self) -> Tuple[float, float]:
        """Perform one-way ANOVA"""
        groups = [np.array(self.data[name]) for name in ['PID+Fuzzy', 'PD', 'PID']]
        f_stat, p_value = stats.f_oneway(*groups)
        return f_stat, p_value

    def compute_eta_squared(self) -> float:
        """Compute eta-squared effect size for ANOVA"""
        groups = [np.array(self.data[name]) for name in ['PID+Fuzzy', 'PD', 'PID']]

        # Overall mean
        all_values = np.concatenate(groups)
        grand_mean = np.mean(all_values)

        # Sum of squares between groups
        ss_between = sum(len(g) * (np.mean(g) - grand_mean)**2 for g in groups)

        # Sum of squares total
        ss_total = np.sum((all_values - grand_mean)**2)

        eta_squared = ss_between / ss_total if ss_total > 0 else 0
        return eta_squared

    def compute_cohens_d(self, group1: str, group2: str) -> float:
        """Compute Cohen's d effect size between two groups"""
        x1 = np.array(self.data[group1])
        x2 = np.array(self.data[group2])

        n1, n2 = len(x1), len(x2)
        var1, var2 = np.var(x1, ddof=1), np.var(x2, ddof=1)

        # Pooled standard deviation
        pooled_std = np.sqrt(((n1 - 1) * var1 + (n2 - 1) * var2) / (n1 + n2 - 2))

        if pooled_std == 0:
            return 0

        d = (np.mean(x1) - np.mean(x2)) / pooled_std
        return d

    def interpret_cohens_d(self, d: float) -> str:
        """Interpret Cohen's d effect size"""
        d_abs = abs(d)
        if d_abs < 0.2:
            return "negligible"
        elif d_abs < 0.5:
            return "small"
        elif d_abs < 0.8:
            return "medium"
        else:
            return "large"

    def run_tukey_hsd(self) -> str:
        """Perform Tukey HSD post-hoc test"""
        if not HAS_STATSMODELS:
            return "Tukey HSD not available (statsmodels not installed)"

        # Prepare data for Tukey
        values = []
        groups = []
        for name in ['PID+Fuzzy', 'PD', 'PID']:
            for v in self.data[name]:
                values.append(v)
                groups.append(name)

        result = pairwise_tukeyhsd(values, groups, alpha=0.05)
        return str(result)

    def analyze(self) -> ANOVAResults:
        """Run complete ANOVA analysis"""
        n_samples = self.load_all_runs()

        if n_samples == 0:
            raise ValueError("No data loaded!")

        # Run ANOVA
        f_stat, p_value = self.run_anova()

        # Effect sizes
        eta_sq = self.compute_eta_squared()

        cohens_d = {}
        for g1, g2 in [('PID+Fuzzy', 'PID'), ('PID+Fuzzy', 'PD'), ('PD', 'PID')]:
            key = f"{g1} vs {g2}"
            cohens_d[key] = self.compute_cohens_d(g1, g2)

        # Post-hoc
        tukey = self.run_tukey_hsd()

        # Means and stds
        means = {name: np.mean(vals) for name, vals in self.data.items()}
        stds = {name: np.std(vals, ddof=1) for name, vals in self.data.items()}

        return ANOVAResults(
            metric_name="RMSE Total (m)",
            n_per_group=n_samples,
            group_means=means,
            group_stds=stds,
            f_statistic=f_stat,
            p_value=p_value,
            eta_squared=eta_sq,
            tukey_results=tukey,
            cohens_d=cohens_d
        )

    def print_results(self, results: ANOVAResults):
        """Print formatted results"""
        print("\n" + "=" * 70)
        print("  ANOVA RESULTS - Controller Comparison")
        print("=" * 70)

        print(f"\nMetric: {results.metric_name}")
        print(f"Sample Size: n={results.n_per_group} per controller")

        print("\n" + "-" * 50)
        print(f"{'Controller':<15} {'Mean':>12} {'Std':>12}")
        print("-" * 50)

        order = ['PID+Fuzzy', 'PD', 'PID']
        for name in order:
            mean = results.group_means[name]
            std = results.group_stds[name]
            print(f"{name:<15} {mean:>12.4f} {std:>12.4f}")

        print("\n" + "=" * 70)
        print("  ONE-WAY ANOVA")
        print("=" * 70)
        print(f"\nF-statistic: {results.f_statistic:.4f}")
        print(f"p-value: {results.p_value:.6f}")
        print(f"Eta-squared (eta^2): {results.eta_squared:.4f}")

        significance = "***" if results.p_value < 0.001 else \
                      "**" if results.p_value < 0.01 else \
                      "*" if results.p_value < 0.05 else "ns"
        print(f"Significance: {significance}")

        # Interpret eta-squared
        if results.eta_squared < 0.01:
            eta_interp = "negligible"
        elif results.eta_squared < 0.06:
            eta_interp = "small"
        elif results.eta_squared < 0.14:
            eta_interp = "medium"
        else:
            eta_interp = "large"
        print(f"Effect size interpretation: {eta_interp}")

        print("\n" + "=" * 70)
        print("  TUKEY HSD POST-HOC TEST (alpha=0.05)")
        print("=" * 70)
        print(results.tukey_results)

        print("\n" + "=" * 70)
        print("  PAIRWISE EFFECT SIZES (Cohen's d)")
        print("=" * 70)
        for comparison, d in results.cohens_d.items():
            interp = self.interpret_cohens_d(d)
            print(f"{comparison}: d = {d:.4f} ({interp})")

        print("\n" + "=" * 70)
        print("  ANALYSIS COMPLETE")
        print("=" * 70)

    def plot_boxplot(self, output_dir: Path):
        """Create box plot visualization"""
        output_dir.mkdir(exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 6))

        order = ['PID+Fuzzy', 'PD', 'PID']
        data_to_plot = [self.data[name] for name in order]
        colors = [self.COLORS[name] for name in order]

        bp = ax.boxplot(data_to_plot, labels=order, patch_artist=True)

        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        # Add individual points
        for i, (name, values) in enumerate(zip(order, data_to_plot)):
            x = np.random.normal(i + 1, 0.04, size=len(values))
            ax.scatter(x, values, alpha=0.6, color='black', s=30, zorder=3)

        ax.set_ylabel('RMSE Total (m)', fontsize=12)
        ax.set_title('Controller Performance Comparison\n(One-Way ANOVA)', fontsize=14, fontweight='bold')
        ax.grid(True, axis='y', alpha=0.3)

        # Add significance annotation if significant
        f_stat, p_value = self.run_anova()
        sig_text = f"F = {f_stat:.2f}, p = {p_value:.4f}"
        if p_value < 0.05:
            sig_text += " *"
        ax.text(0.02, 0.98, sig_text, transform=ax.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plot_path = output_dir / 'anova_boxplot.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"\nSaved: {plot_path}")
        plt.close()

    def save_csv(self, results: ANOVAResults, output_dir: Path):
        """Save results to CSV"""
        output_dir.mkdir(exist_ok=True)

        # Summary statistics
        rows = []
        for name in ['PID+Fuzzy', 'PD', 'PID']:
            rows.append({
                'controller': name,
                'n': results.n_per_group,
                'mean_rmse': results.group_means[name],
                'std_rmse': results.group_stds[name]
            })

        df_summary = pd.DataFrame(rows)
        summary_path = output_dir / 'anova_summary.csv'
        df_summary.to_csv(summary_path, index=False)
        print(f"Saved: {summary_path}")

        # ANOVA statistics
        anova_stats = {
            'f_statistic': [results.f_statistic],
            'p_value': [results.p_value],
            'eta_squared': [results.eta_squared],
            'n_per_group': [results.n_per_group]
        }
        df_anova = pd.DataFrame(anova_stats)
        anova_path = output_dir / 'anova_statistics.csv'
        df_anova.to_csv(anova_path, index=False)
        print(f"Saved: {anova_path}")

        # Raw RMSE values per run
        raw_rows = []
        for name in ['PID+Fuzzy', 'PD', 'PID']:
            for i, rmse in enumerate(self.data[name]):
                raw_rows.append({
                    'controller': name,
                    'run': i + 1,
                    'rmse': rmse
                })
        df_raw = pd.DataFrame(raw_rows)
        raw_path = output_dir / 'anova_raw_data.csv'
        df_raw.to_csv(raw_path, index=False)
        print(f"Saved: {raw_path}")


def main():
    parser = argparse.ArgumentParser(description='ANOVA Statistical Analysis')
    parser.add_argument('data_dir', type=str, help='Base directory with run_X subdirectories')
    args = parser.parse_args()

    analyzer = ANOVAAnalyzer(args.data_dir)

    try:
        results = analyzer.analyze()
        analyzer.print_results(results)

        output_dir = Path(args.data_dir) / 'analysis'
        analyzer.plot_boxplot(output_dir)
        analyzer.save_csv(results, output_dir)

    except Exception as e:
        print(f"Error: {e}")
        raise


if __name__ == '__main__':
    main()
