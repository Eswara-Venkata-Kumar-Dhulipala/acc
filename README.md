# Adaptive Cruise Control (ACC) — Jerk-Limited, Safety-Constrained Simulation

This repository contains a complete Python implementation of a **Jerk-Limited Adaptive Cruise Control (ACC)** system with realistic longitudinal vehicle dynamics. The project demonstrates a full ADAS-style control pipeline without relying on proprietary tools like MATLAB/Simulink.

---

## 🚗 Project Features

### ✔ Dual-Mode ACC Controller
- **Speed Mode**: Tracks a desired set speed using proportional control.
- **Gap Mode**: Maintains a safe time-gap distance using PI control.
- **Hysteresis logic** prevents mode-switching oscillations.

### ✔ Safety Constraint Enforcement
Implements the industry-standard rule:



\[
d_{\text{actual}} \ge d_{\text{desired}} = t_{\text{gap}} v_{\text{ego}} + d_{\min}
\]



If violated, the controller automatically applies safe braking.

### ✔ Jerk-Limited Control
Ensures smooth acceleration transitions:



\[
|a(k) - a(k-1)| \le J_{\max} \Delta t
\]



### ✔ Realistic Vehicle Dynamics
Includes:
- Aerodynamic drag  
- Rolling resistance  
- Engine/brake force saturation  
- Mass-based acceleration dynamics  

---

## 📊 Simulation Outputs

The script generates:
- Ego vs Lead Speed (with settling markers)
- Distance Tracking
- Mode Switching Timeline
- Acceleration Profile
- Jerk Profile

---

## 📈 Performance Metrics

| Metric | Value |
|--------|--------|
| Max deceleration | −0.096 m/s² |
| Max jerk | 1.000 m/s³ |
| Time to 95% set speed | 24.40 s |
| Settling time | 30.50 s |
| Minimum distance margin | 0.000 m |
| Comfort score | 10/10 |

---

## 🧠 Technologies Used
- Python (NumPy, Matplotlib)
- LaTeX (Documentation)
- TikZ (Block Diagrams)

---

## 📁 Repository Structure
acc.py                # Main ACC simulation script
plots/               # Generated figures
docs/                # LaTeX design note
README.md             # Project documentation

---

## 🛠️ How to Run
```bash
python acc.py

