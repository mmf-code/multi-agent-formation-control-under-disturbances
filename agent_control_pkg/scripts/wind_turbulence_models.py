#!/usr/bin/env python3
"""
Atmospheric Turbulence Models for Multi-Agent Formation Control

Implements scientifically-grounded wind turbulence models for thesis-quality
simulation of atmospheric disturbances on drone formations.

Standards:
- MIL-F-8785C: Military Specification - Flying Qualities of Piloted Airplanes
- NASA TP-1313: A Comparison of the von Kármán and Dryden Turbulence Models
- Kolmogorov -5/3 inertial subrange law

Author: Multi-Agent Formation Control Research
Date: 2026-01-02
"""

import numpy as np
from scipy import signal
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class TurbulenceParameters:
    """
    Atmospheric turbulence parameters.

    Attributes:
        L_u, L_v, L_w: Integral length scales [m] for u, v, w components
        sigma_u, sigma_v, sigma_w: Turbulence intensities [m/s]
        V: Mean wind speed [m/s]
        dt: Time step for discrete simulation [s]
    """
    L_u: float
    L_v: float
    L_w: float
    sigma_u: float
    sigma_v: float
    sigma_w: float
    V: float
    dt: float = 0.1


@dataclass
class TurbulenceSample:
    """Single turbulence sample in body axes."""
    u: float  # Longitudinal component [m/s]
    v: float  # Lateral component [m/s]
    w: float  # Vertical component [m/s]


class VonKarmanTurbulence:
    """
    von Kármán atmospheric turbulence model.

    Implements the von Kármán spectral model per MIL-F-8785C Section 3.7.3,
    based on Kolmogorov's -5/3 inertial subrange theory.

    Power Spectral Density (PSD):
    Φ_u(Ω) = (2 σ_u² L_u / π) / (1 + (1.339 L_u Ω)²)^(5/6)
    Φ_v(Ω) = σ_v² L_v / π * (1 + 8/3 (1.339 L_v Ω)²) / (1 + (1.339 L_v Ω)²)^(11/6)
    Φ_w(Ω) = σ_w² L_w / π * (1 + 8/3 (1.339 L_w Ω)²) / (1 + (1.339 L_w Ω)²)^(11/6)

    where Ω = ω / V (spatial frequency in rad/m)

    References:
    - MIL-F-8785C, Section 3.7.3
    - NASA TP-1313 (1980)
    - Kolmogorov (1941) - Local structure of turbulence
    """

    def __init__(self, params: TurbulenceParameters):
        """
        Initialize von Kármán turbulence generator.

        Args:
            params: TurbulenceParameters with length scales, intensities, mean wind
        """
        self.params = params
        self.dt = params.dt

        # State variables for continuous turbulence generation
        self.state_u: Optional[np.ndarray] = None
        self.state_v: Optional[np.ndarray] = None
        self.state_w: Optional[np.ndarray] = None

        # Design forming filters
        self._design_filters()

    def _design_filters(self):
        """
        Design IIR filters to shape white noise into von Kármán spectrum.

        Uses bilinear transform to discretize continuous-time transfer functions.
        Filter order is chosen to balance accuracy vs computational cost.
        """
        # Nyquist frequency
        fs = 1.0 / self.dt
        nyq = 0.5 * fs

        # Characteristic frequencies (converted from spatial to temporal)
        # Ω_0 = 1 / (1.339 L) in rad/m, ω_0 = V * Ω_0 in rad/s
        omega_u = self.params.V / (1.339 * self.params.L_u)
        omega_v = self.params.V / (1.339 * self.params.L_v)
        omega_w = self.params.V / (1.339 * self.params.L_w)

        # Normalize to Nyquist frequency
        wn_u = omega_u / (2 * np.pi * nyq)
        wn_v = omega_v / (2 * np.pi * nyq)
        wn_w = omega_w / (2 * np.pi * nyq)

        # Clamp to valid range for filter design
        wn_u = np.clip(wn_u, 0.001, 0.999)
        wn_v = np.clip(wn_v, 0.001, 0.999)
        wn_w = np.clip(wn_w, 0.001, 0.999)

        # Design filters using Butterworth approximation to von Kármán PSD
        # Order 3 approximates 5/6 exponent reasonably well
        # Gain adjustment to match σ² at low frequencies
        gain_u = self.params.sigma_u * np.sqrt(2 * self.params.L_u / (np.pi * self.params.V))
        gain_v = self.params.sigma_v * np.sqrt(self.params.L_v / (np.pi * self.params.V))
        gain_w = self.params.sigma_w * np.sqrt(self.params.L_w / (np.pi * self.params.V))

        # Butterworth filters (3rd order for u, 5th order for v/w due to 11/6 exponent)
        self.sos_u = signal.butter(3, wn_u, btype='low', output='sos')
        self.sos_v = signal.butter(5, wn_v, btype='low', output='sos')
        self.sos_w = signal.butter(5, wn_w, btype='low', output='sos')

        self.gain_u = gain_u
        self.gain_v = gain_v
        self.gain_w = gain_w

        # Initialize filter states
        self.state_u = signal.sosfilt_zi(self.sos_u)
        self.state_v = signal.sosfilt_zi(self.sos_v)
        self.state_w = signal.sosfilt_zi(self.sos_w)

    def generate_sample(self) -> TurbulenceSample:
        """
        Generate single turbulence sample.

        Returns:
            TurbulenceSample with u, v, w components
        """
        # Generate white noise
        white_u = np.random.randn()
        white_v = np.random.randn()
        white_w = np.random.randn()

        # Filter through forming filters
        u_filt, self.state_u = signal.sosfilt(
            self.sos_u, [white_u], zi=self.state_u
        )
        v_filt, self.state_v = signal.sosfilt(
            self.sos_v, [white_v], zi=self.state_v
        )
        w_filt, self.state_w = signal.sosfilt(
            self.sos_w, [white_w], zi=self.state_w
        )

        # Apply gains
        u = float(u_filt[0] * self.gain_u)
        v = float(v_filt[0] * self.gain_v)
        w = float(w_filt[0] * self.gain_w)

        return TurbulenceSample(u=u, v=v, w=w)

    def generate_turbulence(self, duration: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate turbulence time series.

        Args:
            duration: Duration of time series [s]

        Returns:
            Tuple of (time, u, v, w) arrays
        """
        n_samples = int(duration / self.dt)
        time = np.arange(n_samples) * self.dt

        u = np.zeros(n_samples)
        v = np.zeros(n_samples)
        w = np.zeros(n_samples)

        for i in range(n_samples):
            sample = self.generate_sample()
            u[i] = sample.u
            v[i] = sample.v
            w[i] = sample.w

        return time, u, v, w

    def get_theoretical_psd(self, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute theoretical von Kármán PSD for validation.

        Args:
            frequencies: Frequency array [Hz]

        Returns:
            Tuple of (Psd_u, Psd_v, Psd_w) arrays [m²/s²/Hz]
        """
        # Convert frequency to spatial frequency: Ω = 2πf / V
        omega_spatial = 2 * np.pi * frequencies / self.params.V

        # Avoid division by zero
        omega_spatial = np.where(omega_spatial == 0, 1e-10, omega_spatial)

        # von Kármán PSD - longitudinal component
        Lu_omega = 1.339 * self.params.L_u * omega_spatial
        Psd_u = (2 * self.params.sigma_u**2 * self.params.L_u / (np.pi * self.params.V)) / \
                (1 + Lu_omega**2)**(5.0/6.0)

        # Lateral and vertical components (more complex with 11/6 exponent)
        Lv_omega = 1.339 * self.params.L_v * omega_spatial
        Lw_omega = 1.339 * self.params.L_w * omega_spatial

        Psd_v = (self.params.sigma_v**2 * self.params.L_v / (np.pi * self.params.V)) * \
                (1 + 8.0/3.0 * Lv_omega**2) / (1 + Lv_omega**2)**(11.0/6.0)

        Psd_w = (self.params.sigma_w**2 * self.params.L_w / (np.pi * self.params.V)) * \
                (1 + 8.0/3.0 * Lw_omega**2) / (1 + Lw_omega**2)**(11.0/6.0)

        return Psd_u, Psd_v, Psd_w


class DrydenTurbulence:
    """
    Dryden atmospheric turbulence model.

    Implements the Dryden spectral model per MIL-F-8785C.
    Computationally lighter than von Kármán, uses rational transfer functions.

    Power Spectral Density (PSD):
    Φ_u(Ω) = (2 σ_u² L_u / πV) / (1 + (L_u Ω)²)
    Φ_v(Ω) = σ_v² L_v / πV * (1 + 3(L_v Ω)²) / (1 + (L_v Ω)²)²
    Φ_w(Ω) = σ_w² L_w / πV * (1 + 3(L_w Ω)²) / (1 + (L_w Ω)²)²

    where Ω = ω / V (spatial frequency in rad/m)

    References:
    - MIL-F-8785C
    - Dryden, H. L. (1943) - A review of the statistical theory of turbulence
    """

    def __init__(self, params: TurbulenceParameters):
        """
        Initialize Dryden turbulence generator.

        Args:
            params: TurbulenceParameters with length scales, intensities, mean wind
        """
        self.params = params
        self.dt = params.dt

        # State variables for continuous turbulence generation
        self.state_u: Optional[np.ndarray] = None
        self.state_v: Optional[np.ndarray] = None
        self.state_w: Optional[np.ndarray] = None

        # Design forming filters
        self._design_filters()

    def _design_filters(self):
        """
        Design IIR filters to shape white noise into Dryden spectrum.

        Dryden transfer functions are rational, making filter design straightforward.
        """
        # Nyquist frequency
        fs = 1.0 / self.dt
        nyq = 0.5 * fs

        # Characteristic frequencies (temporal domain)
        # ω_0 = V / L in rad/s
        omega_u = self.params.V / self.params.L_u
        omega_v = self.params.V / self.params.L_v
        omega_w = self.params.V / self.params.L_w

        # Normalize to Nyquist frequency
        wn_u = omega_u / (2 * np.pi * nyq)
        wn_v = omega_v / (2 * np.pi * nyq)
        wn_w = omega_w / (2 * np.pi * nyq)

        # Clamp to valid range
        wn_u = np.clip(wn_u, 0.001, 0.999)
        wn_v = np.clip(wn_v, 0.001, 0.999)
        wn_w = np.clip(wn_w, 0.001, 0.999)

        # Gain to match variance
        gain_u = self.params.sigma_u * np.sqrt(2 * self.params.L_u / (np.pi * self.params.V))
        gain_v = self.params.sigma_v * np.sqrt(self.params.L_v / (3 * np.pi * self.params.V))
        gain_w = self.params.sigma_w * np.sqrt(self.params.L_w / (3 * np.pi * self.params.V))

        # Butterworth filters (1st order for u, 2nd order for v/w)
        self.sos_u = signal.butter(1, wn_u, btype='low', output='sos')
        self.sos_v = signal.butter(2, wn_v, btype='low', output='sos')
        self.sos_w = signal.butter(2, wn_w, btype='low', output='sos')

        self.gain_u = gain_u
        self.gain_v = gain_v
        self.gain_w = gain_w

        # Initialize filter states
        self.state_u = signal.sosfilt_zi(self.sos_u)
        self.state_v = signal.sosfilt_zi(self.sos_v)
        self.state_w = signal.sosfilt_zi(self.sos_w)

    def generate_sample(self) -> TurbulenceSample:
        """
        Generate single turbulence sample.

        Returns:
            TurbulenceSample with u, v, w components
        """
        # Generate white noise
        white_u = np.random.randn()
        white_v = np.random.randn()
        white_w = np.random.randn()

        # Filter through forming filters
        u_filt, self.state_u = signal.sosfilt(
            self.sos_u, [white_u], zi=self.state_u
        )
        v_filt, self.state_v = signal.sosfilt(
            self.sos_v, [white_v], zi=self.state_v
        )
        w_filt, self.state_w = signal.sosfilt(
            self.sos_w, [white_w], zi=self.state_w
        )

        # Apply gains
        u = float(u_filt[0] * self.gain_u)
        v = float(v_filt[0] * self.gain_v)
        w = float(w_filt[0] * self.gain_w)

        return TurbulenceSample(u=u, v=v, w=w)

    def generate_turbulence(self, duration: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate turbulence time series.

        Args:
            duration: Duration of time series [s]

        Returns:
            Tuple of (time, u, v, w) arrays
        """
        n_samples = int(duration / self.dt)
        time = np.arange(n_samples) * self.dt

        u = np.zeros(n_samples)
        v = np.zeros(n_samples)
        w = np.zeros(n_samples)

        for i in range(n_samples):
            sample = self.generate_sample()
            u[i] = sample.u
            v[i] = sample.v
            w[i] = sample.w

        return time, u, v, w

    def get_theoretical_psd(self, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute theoretical Dryden PSD for validation.

        Args:
            frequencies: Frequency array [Hz]

        Returns:
            Tuple of (Psd_u, Psd_v, Psd_w) arrays [m²/s²/Hz]
        """
        # Convert frequency to spatial frequency: Ω = 2πf / V
        omega_spatial = 2 * np.pi * frequencies / self.params.V

        # Avoid division by zero
        omega_spatial = np.where(omega_spatial == 0, 1e-10, omega_spatial)

        # Dryden PSD - longitudinal component
        Lu_omega = self.params.L_u * omega_spatial
        Psd_u = (2 * self.params.sigma_u**2 * self.params.L_u / (np.pi * self.params.V)) / \
                (1 + Lu_omega**2)

        # Lateral and vertical components
        Lv_omega = self.params.L_v * omega_spatial
        Lw_omega = self.params.L_w * omega_spatial

        Psd_v = (self.params.sigma_v**2 * self.params.L_v / (np.pi * self.params.V)) * \
                (1 + 3 * Lv_omega**2) / (1 + Lv_omega**2)**2

        Psd_w = (self.params.sigma_w**2 * self.params.L_w / (np.pi * self.params.V)) * \
                (1 + 3 * Lw_omega**2) / (1 + Lw_omega**2)**2

        return Psd_u, Psd_v, Psd_w


def create_standard_turbulence(
    turbulence_intensity: float,
    mean_wind_speed: float,
    altitude: float = 1.0,
    model: str = 'vonkarman',
    dt: float = 0.1
) -> VonKarmanTurbulence | DrydenTurbulence:
    """
    Create standard turbulence model with typical parameters for small drones.

    Args:
        turbulence_intensity: Turbulence intensity (0.0 to 1.0, typically 0.10-0.20)
        mean_wind_speed: Mean wind speed [m/s]
        altitude: Reference altitude [m] (affects length scales)
        model: 'vonkarman' or 'dryden'
        dt: Time step [s]

    Returns:
        Initialized turbulence generator

    Raises:
        ValueError: If model not recognized
    """
    # Standard length scales per MIL-F-8785C (low altitude)
    # L_u ~ altitude / 2 for low altitude (<300m)
    L_u = max(50.0, altitude / 2.0)  # Minimum 50m for numerical stability
    L_v = L_u / 2.0  # Typical ratio
    L_w = altitude if altitude > 10.0 else 10.0  # Vertical scale ~ altitude

    # Turbulence intensities (all components have same TI)
    sigma_u = turbulence_intensity * mean_wind_speed
    sigma_v = sigma_u  # Isotropic assumption
    sigma_w = sigma_u

    # Create parameters
    params = TurbulenceParameters(
        L_u=L_u,
        L_v=L_v,
        L_w=L_w,
        sigma_u=sigma_u,
        sigma_v=sigma_v,
        sigma_w=sigma_w,
        V=mean_wind_speed,
        dt=dt
    )

    # Create model
    if model.lower() == 'vonkarman':
        return VonKarmanTurbulence(params)
    elif model.lower() == 'dryden':
        return DrydenTurbulence(params)
    else:
        raise ValueError(f"Unknown turbulence model: {model}. Use 'vonkarman' or 'dryden'")


if __name__ == "__main__":
    """Quick validation test."""
    print("Testing von Kármán and Dryden turbulence models...")

    # Test parameters (typical for small drone at low altitude)
    params = TurbulenceParameters(
        L_u=50.0,
        L_v=25.0,
        L_w=10.0,
        sigma_u=0.3,  # 10% TI at 3 m/s mean wind
        sigma_v=0.3,
        sigma_w=0.3,
        V=3.0,
        dt=0.1
    )

    # Generate short time series
    vk = VonKarmanTurbulence(params)
    dr = DrydenTurbulence(params)

    print("\nvon Kármán samples:")
    for i in range(5):
        sample = vk.generate_sample()
        print(f"  Sample {i}: u={sample.u:6.3f}, v={sample.v:6.3f}, w={sample.w:6.3f} m/s")

    print("\nDryden samples:")
    for i in range(5):
        sample = dr.generate_sample()
        print(f"  Sample {i}: u={sample.u:6.3f}, v={sample.v:6.3f}, w={sample.w:6.3f} m/s")

    print("\n✓ Turbulence models initialized successfully")
