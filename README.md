# 🚗 Adaptive Cruise Control (ACC) — Jerk‑Limited, Safety‑Constrained Simulation

This repository contains a complete Python implementation of a **Jerk‑Limited Adaptive Cruise Control (ACC)** system with realistic longitudinal vehicle dynamics.  
The project demonstrates a full ADAS‑style control pipeline using **open‑source tools only**, making it fully reproducible and license‑safe.

---

## ⭐ Project Highlights

### ✔ Dual‑Mode ACC Controller
- **Speed Mode** — tracks a desired set speed using proportional control  
- **Gap Mode** — maintains a safe time‑gap distance using PI control  
- **Hysteresis logic** prevents rapid mode switching  

### ✔ Safety Constraint Enforcement
Implements the industry‑standard rule:



\[
d_{\text{actual}} \ge d_{\text{desired}} = t_{\text{gap}} v_{\text{ego}} + d_{\min}
\]



If violated, the controller automatically applies safe braking.

### ✔ Jerk‑Limited Control
Ensures smooth acceleration transitions:



\[
|a(k) - a(k-1)| \le J_{\max} \Delta t
\]



This improves passenger comfort and prevents aggressive acceleration changes.

### ✔ Realistic Vehicle Dynamics
Includes:
- Aerodynamic drag  
- Rolling resistance  
- Engine/brake force saturation  
- Mass‑based acceleration dynamics  

---

## 📊 Simulation Outputs

The script generates the following plots:

- **Ego vs Lead Speed**  
- **Distance Tracking**  
- **Mode Switching Timeline**  
- **Acceleration Profile**  
- **Jerk Profile**  
- **Settling Time and 95% Speed Markers**

These visualizations help analyze controller behavior, stability, and comfort.

---

## 📈 Performance Metrics

| Metric | Value | Meaning |
|--------|--------|---------|
| Max deceleration | −0.096 m/s² | Very smooth braking |
| Max jerk | 1.000 m/s³ | Within comfort limits |
| Time to 95% set speed | 24.40 s | Smooth acceleration |
| Settling time | 30.50 s | Stable before Gap Mode |
| Minimum distance margin | 0.000 m | Safety constraint satisfied |
| Comfort score | 10/10 | Excellent ride comfort |

---

## 🧠 Technologies Used
- **Python** (NumPy, Matplotlib)  
- **LaTeX** for documentation  
- **TikZ** for block diagrams  
- **GitHub** for version control  

No proprietary or paid tools required.

---

## 📁 Repository Structure
acc.py                # Main ACC simulation script
plots/               # Generated figures
docs/                # LaTeX design note
README.md             # Project documentation

---

## ▶️ How to Run

```bash
python acc.py
