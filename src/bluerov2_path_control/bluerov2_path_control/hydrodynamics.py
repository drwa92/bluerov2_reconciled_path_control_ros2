#!/usr/bin/env python3
"""Hydrodynamic helper models for BlueROV2 virtual-wrench reconstruction.

The coefficients mirror commonly used BlueROV2 heavy/classic parameter sets.
The observer is intentionally lightweight: it reconstructs a 4-DOF wrench
[X, Y, Z, N]^T from body velocities [u, v, w, r]^T.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class HydroCoefficients:
    name: str
    Xdu: float
    Ydv: float
    Zdw: float
    Kdp: float
    Mdq: float
    Ndr: float
    Xu: float
    Yv: float
    Zw: float
    Kp: float
    Mq: float
    Nr: float
    Xuu: float
    Yvv: float
    Zww: float
    Kpp: float
    Mqq: float
    Nrr: float


HYDRO_MODELS = {
    # BlueROV2 heavy: Wu, 6-DoF modelling and control of a ROV.
    "heavy": HydroCoefficients(
        name="heavy",
        Xdu=5.5, Ydv=12.7, Zdw=14.57, Kdp=0.12, Mdq=0.12, Ndr=0.12,
        Xu=4.03, Yv=6.22, Zw=5.18, Kp=0.07, Mq=0.07, Nr=0.07,
        Xuu=18.18, Yvv=21.66, Zww=36.99, Kpp=1.55, Mqq=1.55, Nrr=1.55,
    ),
    # BlueROV2 classic: Li et al. hydrodynamic characterization.
    "classic": HydroCoefficients(
        name="classic",
        Xdu=5.5, Ydv=12.7, Zdw=14.57, Kdp=0.12, Mdq=0.12, Ndr=0.12,
        Xu=1.31, Yv=9.14, Zw=2.015, Kp=0.07, Mq=0.07, Nr=0.0,
        Xuu=38.17, Yvv=129.6, Zww=243.2, Kpp=1.55, Mqq=1.55, Nrr=4.86,
    ),
}


class FourDofWrenchObserver:
    """Model-aided 4-DOF wrench reconstruction.

    It estimates
        tau_hat = M_eff * nu_dot + D(nu) * nu
    for nu=[u,v,w,r] and tau=[X,Y,Z,N].

    The sign convention follows the controller's allocation convention. This is
    an observer for diagnostics and reconciliation, not a high-fidelity CFD model.
    """

    def __init__(
        self,
        model_name: str = "heavy",
        mass: float = 10.0,
        izz: float = 0.269,
        lowpass_alpha: float = 0.25,
        derivative_clip: float = 5.0,
    ) -> None:
        name = str(model_name).strip().lower()
        if name not in HYDRO_MODELS:
            name = "heavy"
        self.coeff = HYDRO_MODELS[name]
        self.mass = float(mass)
        self.izz = float(izz)
        self.lowpass_alpha = float(np.clip(lowpass_alpha, 0.0, 1.0))
        self.derivative_clip = float(max(derivative_clip, 1e-9))
        self.prev_nu: np.ndarray | None = None
        self.tau_filt = np.zeros(4, dtype=float)

    def reset(self) -> None:
        self.prev_nu = None
        self.tau_filt[:] = 0.0

    def effective_mass(self) -> np.ndarray:
        c = self.coeff
        return np.array([
            self.mass + c.Xdu,
            self.mass + c.Ydv,
            self.mass + c.Zdw,
            self.izz + c.Ndr,
        ], dtype=float)

    def damping(self, nu: np.ndarray) -> np.ndarray:
        u, v, w, r = [float(x) for x in nu]
        c = self.coeff
        return np.array([
            c.Xu * u + c.Xuu * abs(u) * u,
            c.Yv * v + c.Yvv * abs(v) * v,
            c.Zw * w + c.Zww * abs(w) * w,
            c.Nr * r + c.Nrr * abs(r) * r,
        ], dtype=float)

    def update(self, nu: np.ndarray, dt: float) -> np.ndarray:
        nu = np.asarray(nu, dtype=float).reshape(4)
        dt = float(max(dt, 1e-6))
        if self.prev_nu is None:
            self.prev_nu = nu.copy()
            return self.tau_filt.copy()

        nu_dot = (nu - self.prev_nu) / dt
        nu_dot = np.clip(nu_dot, -self.derivative_clip, self.derivative_clip)
        tau_raw = self.effective_mass() * nu_dot + self.damping(self.prev_nu)
        a = self.lowpass_alpha
        self.tau_filt = (1.0 - a) * self.tau_filt + a * tau_raw
        self.prev_nu = nu.copy()
        return self.tau_filt.copy()
