import numpy as np
import matplotlib.pyplot as plt

dt = 0.1
time = np.arange(0, 60, dt)

# ACC parameters
v_set = 25.0          # m/s
time_gap = 1.5
d_min = 5.0

enter_gap_margin = 0.0
exit_gap_margin  = 8.0

# Speed: P only (no overshoot to set speed)
kp_v = 0.15

# Distance: PI
kp_d, ki_d = 0.3, 0.06

# Vehicle parameters (longitudinal model)
m = 1500.0           # kg
rho = 1.225          # kg/m^3 (air density)
Cd = 0.32
A = 2.2              # m^2 (frontal area)
Cr = 0.01
g = 9.81             # m/s^2

F_eng_max   = 4000.0   # N (max driving force)
F_brake_max = 8000.0   # N (max braking force magnitude)

# Controller accel limits (request-level)
ACC_MAX_ACCEL = 1.5    # m/s^2
ACC_MIN_ACCEL = -3.0   # m/s^2

# Jerk limit
J_MAX = 1.0            # m/s^3

# Initial conditions
v_ego = 0.0
v_lead = 20.0
d_rel = 60.0

ego_speeds = []
lead_speeds = []
distances = []
desired_distances = []
modes = []
acc_hist = []
jerk_hist = []

int_d = 0.0
acc_prev = 0.0
mode = "Speed"

for k, t in enumerate(time):

    # Lead vehicle profile
    if 20 < t < 30:
        v_lead = max(12.0, v_lead - 0.3 * dt)
    elif 30 <= t < 40:
        v_lead = 12.0
    elif 40 <= t < 50:
        v_lead = min(20.0, v_lead + 0.3 * dt)
    else:
        v_lead = 20.0

    # Desired safe distance
    d_des = time_gap * v_ego + d_min

    # Mode logic with hysteresis
    if mode == "Speed":
        if d_rel < d_des + enter_gap_margin:
            mode = "Gap"
    else:  # Gap
        if d_rel > d_des + exit_gap_margin:
            mode = "Speed"

    # Base controllers (acc_cmd is desired longitudinal accel)
    if mode == "Speed":
        err_v = v_set - v_ego
        acc_cmd = kp_v * err_v
        int_d = 0.0
    else:
        err_d = d_rel - d_des
        int_d += err_d * dt
        int_d = np.clip(int_d, -50, 50)
        acc_cmd = kp_d * err_d + ki_d * int_d

    # Safety override: never allow d_rel < d_des
    if d_rel <= d_des:
        mode = "Gap"
        acc_cmd = ACC_MIN_ACCEL

    # Clip requested acceleration to comfort bounds
    acc_cmd = np.clip(acc_cmd, ACC_MIN_ACCEL, ACC_MAX_ACCEL)

    # ---- JERK LIMITING on requested acceleration ----
    a_des = acc_cmd
    delta_a = a_des - acc_prev
    max_delta_a = J_MAX * dt
    delta_a = np.clip(delta_a, -max_delta_a, max_delta_a)
    acc_req = acc_prev + delta_a    # jerk-limited requested acceleration
    # -------------------------------------------------

    # Longitudinal forces
    F_drag = 0.5 * rho * Cd * A * v_ego**2
    F_roll = Cr * m * g

    # Required net force to achieve acc_req
    F_req = m * acc_req + F_drag + F_roll

    # Engine/brake saturation
    F_req = np.clip(F_req, -F_brake_max, F_eng_max)

    # Actual acceleration from saturated force
    acc = (F_req - F_drag - F_roll) / m

    # Update jerk history (approx)
    if k == 0:
        jerk = 0.0
    else:
        jerk = (acc - acc_hist[-1]) / dt

    acc_prev = acc

    # Update ego speed
    v_ego += acc * dt
    v_ego = max(0.0, v_ego)

    # Enforce no overshoot of set speed
    if v_ego > v_set:
        v_ego = v_set

    # Update distance
    d_rel += (v_lead - v_ego) * dt
    if d_rel < d_des:
        d_rel = d_des

    # Log
    ego_speeds.append(v_ego)
    lead_speeds.append(v_lead)
    distances.append(d_rel)
    desired_distances.append(d_des)
    modes.append(mode)
    acc_hist.append(acc)
    jerk_hist.append(jerk)

mode_numeric = [1 if m == "Gap" else 0 for m in modes]

# Plots
plt.figure()
plt.plot(time, ego_speeds, label="Ego Speed")
plt.plot(time, lead_speeds, '--', label="Lead Speed")
plt.axhline(v_set, color='gray', linestyle='--', label="Set Speed")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.title("Jerk-Limited ACC with Longitudinal Model: Ego vs Lead Speed")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(time, distances, label="Actual Distance")
plt.plot(time, desired_distances, '--', label="Desired Distance")
plt.xlabel("Time [s]")
plt.ylabel("Distance [m]")
plt.title("Jerk-Limited ACC with Longitudinal Model: Distance Tracking")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure()
plt.step(time, mode_numeric, where='post')
plt.yticks([0, 1], ["Speed Mode", "Gap Mode"])
plt.xlabel("Time [s]")
plt.ylabel("Mode")
plt.title("Jerk-Limited ACC with Longitudinal Model: Mode Switching")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(time, acc_hist)
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.title("Jerk-Limited ACC with Longitudinal Model: Acceleration Profile")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(time, jerk_hist)
plt.xlabel("Time [s]")
plt.ylabel("Jerk [m/s³]")
plt.title("Jerk-Limited ACC with Longitudinal Model: Jerk Profile")
plt.grid(True)
plt.tight_layout()
plt.show()

import numpy as np

acc_arr = np.array(acc_hist)
jerk_arr = np.array(jerk_hist)
v_arr = np.array(ego_speeds)
d_arr = np.array(distances)
ddes_arr = np.array(desired_distances)

# 1) Max deceleration (negative)
max_decel = np.min(acc_arr)

# 2) Max jerk (absolute)
max_jerk = np.max(np.abs(jerk_arr))

# 3) Settling time (free-road, before Gap Mode)
eps = 0.5  # m/s band
# index when Gap Mode first activates
gap_start_idx = next((i for i, m in enumerate(modes) if m == "Gap"), len(time)-1)

within_band = np.abs(v_arr[:gap_start_idx] - v_set) < eps

settling_time = None
for i in range(gap_start_idx):
    if within_band[i] and np.all(within_band[i:gap_start_idx]):
        settling_time = time[i]
        break

# 4) Time to reach 95% of set speed
reach_idx = np.where(v_arr >= 0.95 * v_set)[0]
if len(reach_idx) > 0:
    t_reach_95 = time[reach_idx[0]]
else:
    t_reach_95 = None

# 5) Minimum distance margin
margin_arr = d_arr - ddes_arr
min_margin = np.min(margin_arr)

# 6) Comfort score (0–10)
# Define "ideal" bounds for comfort:
MAX_ACCEPTABLE_DECEL = -2.0    # m/s^2 (more negative is uncomfortable)
MAX_ACCEPTABLE_JERK  = 2.0    # m/s^3

# Normalize decel comfort: 1.0 when >= MAX_ACCEPTABLE_DECEL, 0 when <= -4 m/s^2
decel_score = np.clip((max_decel - (-4.0)) / (MAX_ACCEPTABLE_DECEL - (-4.0)), 0.0, 1.0)

# Normalize jerk comfort: 1.0 when <= MAX_ACCEPTABLE_JERK, 0 when >= 4 m/s^3
jerk_score = np.clip((4.0 - max_jerk) / (4.0 - MAX_ACCEPTABLE_JERK), 0.0, 1.0)

comfort_score = 10.0 * (0.5 * decel_score + 0.5 * jerk_score)

print(f"Max deceleration: {max_decel:.3f} m/s^2")
print(f"Max jerk:         {max_jerk:.3f} m/s^3")
if settling_time is None:
    print("Settling time:    Not fully settled before Gap Mode")
else:
    print(f"Settling time:    {settling_time:.2f} s")
if t_reach_95 is None:
    print("Time to 95% set speed: Not reached")
else:
    print(f"Time to 95% set speed: {t_reach_95:.2f} s")
print(f"Min distance margin (d_actual - d_desired): {min_margin:.3f} m")
print(f"Comfort score (0–10): {comfort_score:.2f}")

plt.figure()
plt.plot(time, ego_speeds, label="Ego Speed")
plt.plot(time, lead_speeds, '--', label="Lead Speed")
plt.axhline(v_set, color='gray', linestyle='--', label="Set Speed")

# Mark 95% set speed
if t_reach_95 is not None:
    v_95 = 0.95 * v_set
    plt.axhline(v_95, color='green', linestyle=':', linewidth=1, label="95% Set Speed")
    plt.axvline(t_reach_95, color='green', linestyle=':', linewidth=1)
    plt.text(t_reach_95, v_95 + 0.5, f"t95={t_reach_95:.1f}s", color='green')

# Mark settling time (if defined)
if settling_time is not None:
    plt.axvline(settling_time, color='purple', linestyle='--', linewidth=1)
    plt.text(settling_time, v_set - 1.0, f"Ts={settling_time:.1f}s", color='purple')

plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.title("Jerk-Limited ACC: Ego vs Lead Speed with Settling Markers")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
