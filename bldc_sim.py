import numpy as np

class BLDCSim:
    """
    Dynamic simulator for a surface-mounted PMSM/BLDC motor.

    Integrates electrical and mechanical dynamics in the dq rotating frame,
    accepting three-phase voltages as input. Assumes Ld = Lq.

    Dynamics are taken from https://docs.simplefoc.com/foc_theory

    States
    ------
    id, iq      : d/q-axis currents [A]
    θ_m         : mechanical rotor angle [rad]
    ω_m         : mechanical rotor speed [rad/s]

    Intermediate variables
    ----------------------
    ω_e = pole_pairs · ω_m  : electrical angular speed [rad/s]
    θ_e = pole_pairs · θ_m  : electrical rotor angle [rad]
    
    Electrical dynamics (dq frame)
    ------------------------------
        di_d/dt = (v_d - R·i_d + ω_e·L·i_q) / L
        di_q/dt = (v_q - R·i_q - ω_e·L·i_d - ω_e·K_e) / L

    Mechanical dynamics
    -------------------
        τ_e     = K_t · i_q
        dω_m/dt = (τ_e - τ_load) / J
        dθ_m/dt = ω_m

    Parameters
    ----------
    R           : Phase resistance [Ω]
    L           : Phase inductance [H]
    pole_pairs  : Number of pole pairs
    τ_e         : Electromagnetic torque [N·m]
    Kv          : Motor velocity constant [RPM/V]
    K_t         : Torque constant [N·m/A]
    K_e         : Back-EMF constant [V·s/rad]
    J           : Rotor inertia [kg·m²]
    dt          : Simulation timestep [s]
    """

    def __init__(
        self,
        R=10.0,
        L=0.015,
        pole_pairs=7,
        Kv=140,
        J=2e-5,
        dt=1e-5,
    ):
        self.R = R
        self.L = L
        self.pole_pairs = pole_pairs
        self.J = J
        self.dt = dt

        # Torque and back-EMF constants are equal for a PMSM: Kt = Ke = 30/(π·Kv)
        self.Kt = 30 / (np.pi * Kv)
        self.Ke = self.Kt

        # Initial state
        self.id = 0.0
        self.iq = 0.0
        self.theta_m = 0.0
        self.omega_m = 0.0

    def _clarke(self, va, vb, vc):
        """Stationary (abc) → stationary orthogonal (αβ)."""
        v_alpha = (2/3) * (va - 0.5*vb - 0.5*vc)
        v_beta  = (2/3) * (np.sqrt(3)/2 * (vb - vc))
        return v_alpha, v_beta

    def _park(self, v_alpha, v_beta, theta_e):
        """Stationary orthogonal (αβ) → rotating (dq)."""
        c, s = np.cos(theta_e), np.sin(theta_e)
        vd =  c * v_alpha + s * v_beta
        vq = -s * v_alpha + c * v_beta
        return vd, vq

    def step(self, va, vb, vc, tau_load=0.0):
        """
        Advance the simulation by one timestep.

        Parameters
        ----------
        va, vb, vc  : Phase voltages [V]
        tau_load    : External load torque [N·m]

        Returns
        -------
        dict with keys: id, iq, torque, omega_m, theta_m
        """

        # Electrical angle and speed
        theta_e = self.pole_pairs * self.theta_m
        omega_e = self.pole_pairs * self.omega_m

        # Clarke and Park transformations
        v_alpha, v_beta = self._clarke(va, vb, vc)
        vd, vq = self._park(v_alpha, v_beta, theta_e)

        # Electrical dynamics
        did_dt = (vd - self.R * self.id + omega_e * self.L * self.iq) / self.L
        diq_dt = (vq - self.R * self.iq - omega_e * self.L * self.id - omega_e * self.Ke) / self.L

        self.id += did_dt * self.dt
        self.iq += diq_dt * self.dt

        # Mechanical dynamics
        torque = self.Kt * self.iq
        self.omega_m += ((torque - tau_load) / self.J) * self.dt
        self.theta_m += self.omega_m * self.dt

        return {
            "id": self.id,
            "iq": self.iq,
            "torque": torque,
            "omega_m": self.omega_m,
            "theta_m": self.theta_m,
        }
    