#!/usr/bin/env python3
"""Projected virtual-wrench actuator reconciliation utilities."""

from __future__ import annotations

import numpy as np


class ProjectedWrenchReconciler:
    """Bounded, rate-limited virtual-wrench defect estimator.

    Estimate r_hat ≈ tau_actual - B u. The next allocation target should be
    tau_target = tau_cmd - r_hat.
    """

    def __init__(
        self,
        gain: float = 0.08,
        bounds: np.ndarray | list[float] | tuple[float, ...] = (8.0, 8.0, 12.0, 8.0),
        rate_limit: float = 0.8,
        sensor_gate_min: float = 0.25,
    ) -> None:
        self.gain = float(gain)
        self.bounds = np.asarray(bounds, dtype=float).reshape(4)
        self.rate_limit = float(rate_limit)
        self.sensor_gate_min = float(sensor_gate_min)
        self.rhat = np.zeros(4, dtype=float)
        self.last_update_enabled = False

    def reset(self) -> None:
        self.rhat[:] = 0.0
        self.last_update_enabled = False

    def update(self, residual: np.ndarray, dt: float, sensor_confidence: float = 1.0) -> np.ndarray:
        residual = np.asarray(residual, dtype=float).reshape(4)
        dt = float(max(dt, 1e-6))
        if float(sensor_confidence) < self.sensor_gate_min:
            self.last_update_enabled = False
            return self.rhat.copy()

        desired = self.rhat + self.gain * (residual - self.rhat)
        desired = np.clip(desired, -self.bounds, self.bounds)
        max_step = self.rate_limit * dt
        if max_step > 0.0:
            desired = self.rhat + np.clip(desired - self.rhat, -max_step, max_step)
        self.rhat = np.clip(desired, -self.bounds, self.bounds)
        self.last_update_enabled = True
        return self.rhat.copy()
