import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from bldc_sim import BLDCSim

sim = BLDCSim()
log = defaultdict(list)

for i in range(2000):
    t = i * sim.dt
    va = 12 * np.sin(t * 1000)
    vb = 12 * np.sin(t * 1000 - 2 * np.pi / 3)
    vc = 12 * np.sin(t * 1000 + 2 * np.pi / 3)

    state = sim.step(va, vb, vc)

    log["t"].append(t)
    log["va"].append(va)
    log["vb"].append(vb)
    log["vc"].append(vc)
    for key, val in state.items():
        log[key].append(val)

log = {k: np.array(v) for k, v in log.items()}  # convert to numpy arrays
t_ms = log["t"] * 1000

# Plotting
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle("BLDC Motor Simulation Results")

plots = [
    (
        axes[0, 0],
        [("va", "Va"), ("vb", "Vb"), ("vc", "Vc")],
        "Voltage [V]",
        "Phase Voltages",
    ),
    (axes[0, 1], [("id", "Id"), ("iq", "Iq")], "Current [A]", "DQ Currents"),
    (axes[1, 0], [("torque", "Torque")], "Torque [mNm]", "Electromagnetic Torque"),
    (axes[1, 1], [("omega_m", "Speed")], "Speed [RPM]", "Mechanical Speed"),
    (axes[2, 0], [("theta_m", "Angle")], "Angle [deg]", "Mechanical Angle"),
    (axes[2, 1], [("i_mag", "|I|")], "Current [A]", "Total Current Magnitude"),
]

# Derived signals
log["torque"] = log["torque"] * 1000
log["omega_m"] = log["omega_m"] * 30 / np.pi
log["theta_m"] = np.rad2deg(log["theta_m"])
log["i_mag"] = np.hypot(log["id"], log["iq"])

for ax, signals, ylabel, title in plots:
    for key, label in signals:
        ax.plot(t_ms, log[key], label=label)
    ax.set(xlabel="Time [ms]", ylabel=ylabel, title=title)
    ax.grid(True)
    if len(signals) > 1:
        ax.legend()

plt.tight_layout()
plt.show()
