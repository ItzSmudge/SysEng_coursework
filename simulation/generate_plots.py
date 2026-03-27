"""
generate_plots.py
=================
Runs all four Section 5.4 benchmark experiments from scratch and saves
six publication-quality plots.

Dependencies
------------
    pip install numpy scipy matplotlib control

Files required in the same directory
--------------------------------------
    pendulum.py   controllers.py   filters.py

Output files
------------
    plot_eval_a.png   — Experiment 1: Eval A disturbance rejection traces
    plot_eval_b.png   — Experiment 1: Eval B deep fall recovery traces
    plot_eval_c.png   — Experiment 1: Eval C sprint & stop traces
    plot_noise.png    — Experiment 2: noise sensitivity
    plot_window.png   — Experiment 3: filter window sensitivity
    plot_lqr_q.png    — Experiment 4: LQR Q[2,2] sensitivity
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import control as ct
import matplotlib
matplotlib.use('Agg')   # headless — remove if you want interactive windows
import matplotlib.pyplot as plt

from pendulum import Pendulum
from controllers import PIDController, LQRController, TrajectoryPIDController

# ── Colours ──────────────────────────────────────────────────────────────────
C_PID  = '#4472C4'   # blue
C_LQR  = '#ED7D31'   # orange
C_TPID = '#70AD47'   # green

# ── Physical parameters (measured from hardware) ──────────────────────────────
M, m, l, b, DT = 1.3816, 0.05, 0.6, 0.1, 0.001

Q_BASE = np.diag([1.0, 1.0, 100.0, 10.0])
R_BASE = np.array([[0.1]])

PID_BAL = dict(kp_theta=120.0, kd_theta=20.0, ki_theta=5.0,
               kp_x=0.0, kd_x=0.0, ki_x=0.0)

SETTLE_BAND = np.radians(2.0)
SETTLE_WIN  = int(0.5 / DT)   # 500 ms hold window
IMPULSE_STEP = 3000            # disturbance applied at t = 3.0 s

np.random.seed(42)


# ── Metric helpers ────────────────────────────────────────────────────────────
def rms(arr):
    return float(np.sqrt(np.mean(np.array(arr) ** 2)))


def settling_ms(theta_arr, band=SETTLE_BAND, win=SETTLE_WIN, dt=DT):
    """Return time (ms) when theta first enters ±band and stays for win steps."""
    count = 0
    for i, v in enumerate(theta_arr):
        if abs(v) < band:
            count += 1
            if count >= win:
                return (i - win + 1) * dt * 1000
        else:
            count = 0
    return None   # never settled


def time_ax(steps):
    return np.arange(steps) * DT


# ── Simulation runners ────────────────────────────────────────────────────────
def run_balance(ctrl_fac, noise_std=0.01, window_size=50,
                start_angle_deg=0.0, steps=8000, target=[0.0, 0.0]):
    """Balance from a given initial angle, no external disturbances."""
    pend = Pendulum(M=M, m=m, l=l, b=b, dt=DT,
                    mode="0", disturbance_level=0, noise_std_dev=noise_std)
    ctrl = ctrl_fac(window_size)
    state = np.array([0.0, 0.0, np.radians(start_angle_deg), 0.0])
    traj = np.zeros((steps, 4))
    clog = []
    traj[0] = state

    for t in range(1, steps):
        noisy = pend.get_noisy_observation(state)
        u = ctrl.get_action(noisy, target)
        clog.append(u)
        state = pend.step(state, u)
        traj[t] = state

    theta = traj[:, 2]
    peak  = float(np.max(np.abs(theta)))
    st    = settling_ms(theta)

    if st is not None:
        ss     = int(st / (DT * 1000))
        post_t = theta[ss:]
        post_c = np.array(clog[ss:])
        rms_t  = rms(post_t)
        rms_c  = rms(post_c)
        ss_err = float(np.mean(np.abs(post_t)))
    else:
        rms_t  = rms(theta)
        rms_c  = rms(clog)
        ss_err = None

    return {
        'settling_ms':   st,
        'peak_deg':      np.degrees(peak),
        'rms_theta_deg': np.degrees(rms_t),
        'rms_ctrl':      rms_c,
        'ss_error_deg':  np.degrees(ss_err) if ss_err is not None else None,
        'success':       st is not None,
        'traj':          traj,
        'clog':          np.array(clog),
    }


def run_with_impulse(ctrl_fac, noise_std=0.01, window_size=50,
                     impulse_Fd=None, steps=8000):
    """Balance from upright with a single impulse at IMPULSE_STEP."""
    pend = Pendulum(M=M, m=m, l=l, b=b, dt=DT,
                    mode="0", disturbance_level=0, noise_std_dev=noise_std)
    ctrl = ctrl_fac(window_size)
    state = np.array([0.0, 0.0, 0.0, 0.0])
    traj = np.zeros((steps, 4))
    clog = []
    traj[0] = state

    for t in range(1, steps):
        noisy = pend.get_noisy_observation(state)
        u = ctrl.get_action(noisy, [0.0, 0.0])
        clog.append(u)
        Fd = np.array([[0.0, 0.0]])
        if t == IMPULSE_STEP:
            Fd = impulse_Fd.copy()
        state = pend.step(state, u, Fd)
        traj[t] = state

    theta = traj[:, 2]
    post  = theta[IMPULSE_STEP + 1:]
    peak  = float(np.max(np.abs(post)))
    st    = settling_ms(post)

    return {
        'peak_deg':      np.degrees(peak),
        'settling_ms':   st,
        'rms_theta_deg': np.degrees(rms(theta)),
        'rms_ctrl':      rms(clog),
        'traj':          traj,
        'clog':          np.array(clog),
    }


def run_sprint(ctrl_fac, noise_std=0.01, window_size=50, steps=12000):
    """Sprint to 2 m while maintaining balance."""
    pend = Pendulum(M=M, m=m, l=l, b=b, dt=DT,
                    mode="0", disturbance_level=0, noise_std_dev=noise_std)
    ctrl = ctrl_fac(window_size)
    state = np.array([0.0, 0.0, 0.0, 0.0])
    traj = np.zeros((steps, 4))
    clog = []
    traj[0] = state
    target = [2.0, 0.0]

    for t in range(1, steps):
        noisy = pend.get_noisy_observation(state)
        u = ctrl.get_action(noisy, target)
        clog.append(u)
        state = pend.step(state, u)
        traj[t] = state

    x, theta = traj[:, 0], traj[:, 2]
    at_t   = np.abs(x - 2.0) < 0.05
    stable = np.abs(theta) < np.radians(5.0)
    both   = at_t & stable
    win    = int(0.5 / DT)
    arrival_step = None
    count = 0
    for i, v in enumerate(both):
        if v:
            count += 1
        else:
            count = 0
        if count >= win:
            arrival_step = i - win + 1
            break

    end = arrival_step if arrival_step else steps - 1
    return {
        'arrival_s':       (arrival_step * DT) if arrival_step else None,
        'final_x_m':       float(x[end]),
        'pos_error_cm':    (float(x[end]) - 2.0) * 100,
        'final_theta_deg': np.degrees(float(theta[end])),
        'rms_theta_deg':   np.degrees(rms(theta[:end])),
        'max_theta_deg':   np.degrees(float(np.max(np.abs(theta[:end])))),
        'success':         arrival_step is not None,
        'traj':            traj,
        'clog':            np.array(clog),
    }


# ── Controller factories ──────────────────────────────────────────────────────
def pid_fac(ws):
    return PIDController(**PID_BAL, dt=DT, filter_enabled=True, window_size=ws)

def lqr_fac(ws):
    return LQRController(M=M, m=m, l=l, b=b, Q=Q_BASE, R=R_BASE,
                         filter_enabled=True, window_size=ws)

def tpid_fac(ws):
    return TrajectoryPIDController(
        kp_theta=120.0, kd_theta=20.0, ki_theta=5.0,
        kp_x=5.0, kd_x=10.0, ki_x=0.1,
        dt=DT, trajectory_duration=5.0,
        filter_enabled=True, window_size=ws)


# ── Disturbance vectors ───────────────────────────────────────────────────────
SMALL_TAP  = np.array([[0.0,  800.0]])
LARGE_TAP  = np.array([[0.0, 2000.0]])
CART_SHOVE = np.array([[1500.0, 0.0]])


# ============================================================================
# RUN ALL EXPERIMENTS
# ============================================================================
print("Running Experiment 1 — Eval A (disturbance rejection)...")
results_impulse = {}
for cname, cfac in [("PID", pid_fac), ("LQR", lqr_fac)]:
    for dname, dFd in [("small", SMALL_TAP), ("large", LARGE_TAP), ("shove", CART_SHOVE)]:
        np.random.seed(42)
        results_impulse[f"{cname}_{dname}"] = run_with_impulse(
            cfac, noise_std=0.01, window_size=50, impulse_Fd=dFd)

print("Running Experiment 1 — Eval B (deep fall recovery)...")
results_balance = {}
for cname, cfac in [("PID", pid_fac), ("LQR", lqr_fac)]:
    for ang in [5.0, 10.0, 15.0]:
        np.random.seed(0)
        results_balance[f"{cname}_{ang}"] = run_balance(
            cfac, noise_std=0.01, window_size=50, start_angle_deg=ang)

print("Running Experiment 1 — Eval C (sprint)...")
results_sprint = {}
for cname, cfac in [("TPID", tpid_fac), ("LQR", lqr_fac)]:
    np.random.seed(0)
    results_sprint[cname] = run_sprint(cfac, noise_std=0.01, window_size=50)

print("Running Experiment 2 — noise sensitivity...")
noise_levels = [0.0, 0.005, 0.01, 0.05, 0.10]
results_noise = {}
for cname, cfac in [("PID", pid_fac), ("LQR", lqr_fac)]:
    results_noise[cname] = []
    for ns in noise_levels:
        np.random.seed(0)
        results_noise[cname].append(
            run_balance(cfac, noise_std=ns, window_size=50, start_angle_deg=10.0))

print("Running Experiment 3 — filter window sensitivity...")
window_sizes = [1, 5, 10, 20, 50]
results_window = {}
for cname, cfac in [("PID", pid_fac), ("LQR", lqr_fac)]:
    results_window[cname] = []
    for ws in window_sizes:
        np.random.seed(0)
        results_window[cname].append(
            run_balance(cfac, noise_std=0.01, window_size=ws, start_angle_deg=10.0))

print("Running Experiment 4 — LQR Q[2,2] sensitivity...")
q_values = [10.0, 50.0, 100.0, 200.0, 500.0]
results_q = []
for qval in q_values:
    Q_test = np.diag([1.0, 1.0, qval, 10.0])
    I_val  = (1/3) * m * l**2
    denom  = I_val*(M+m) + M*m*l**2
    A = np.array([
        [0, 1, 0, 0],
        [0, -(I_val+m*l**2)*b/denom, (m**2*l**2*9.81)/denom, 0],
        [0, 0, 0, 1],
        [0, -(m*l*b)/denom, (m*l*9.81*(M+m))/denom, 0]
    ])
    B = np.array([[0], [(I_val+m*l**2)/denom], [0], [(m*l)/denom]])
    K, _, _ = ct.lqr(A, B, Q_test, R_BASE)
    k_theta = K[0, 2]

    def lqr_q_fac(ws, Q=Q_test):
        return LQRController(M=M, m=m, l=l, b=b, Q=Q, R=R_BASE,
                             filter_enabled=True, window_size=ws)

    np.random.seed(0)
    r = run_balance(lqr_q_fac, noise_std=0.01, window_size=50, start_angle_deg=10.0)
    results_q.append({'Q22': qval, 'K_theta': k_theta, **r})

print("All experiments done. Generating plots...\n")


# ============================================================================
# PLOT 1 — Eval A: angle traces for all three disturbances
# ============================================================================
fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
fig.suptitle('Simulation: Evaluation A — Disturbance Rejection',
             fontsize=13, fontweight='bold', y=1.02)

dist_info = [
    ('small', 'Small Tap ($F_\\theta$=800 N)'),
    ('large', 'Large Tap ($F_\\theta$=2000 N)'),
    ('shove', 'Cart Shove ($F_x$=1500 N)'),
]

for ax, (dkey, dlabel) in zip(axes, dist_info):
    for cname, col in [('PID', C_PID), ('LQR', C_LQR)]:
        r = results_impulse[f'{cname}_{dkey}']
        t = time_ax(len(r['traj']))
        ax.plot(t, np.degrees(r['traj'][:, 2]), color=col, lw=1.6, label=cname)
    ax.axvline(IMPULSE_STEP * DT, color='red', lw=1.2, ls='--', alpha=0.7, label='Impulse')
    ax.axhline(0,  color='k',    lw=0.7, ls=':')
    ax.axhline( 2, color='grey', lw=0.8, ls='--', alpha=0.5)
    ax.axhline(-2, color='grey', lw=0.8, ls='--', alpha=0.5)
    ax.set_title(dlabel, fontsize=10)
    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_xlim(0, len(r['traj']) * DT)
    ax.grid(True, alpha=0.3, ls='--')

axes[0].set_ylabel('Pendulum Angle (°)', fontsize=9)
axes[0].legend(fontsize=9)
plt.tight_layout()
plt.savefig('plot_eval_a.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_eval_a.png")


# ============================================================================
# PLOT 2 — Eval B: recovery angle traces
# ============================================================================
fig, axes = plt.subplots(1, 3, figsize=(15, 4))
fig.suptitle('Simulation: Evaluation B — Deep Fall Recovery',
             fontsize=13, fontweight='bold', y=1.02)

for ax, ang in zip(axes, [5.0, 10.0, 15.0]):
    for cname, col in [('PID', C_PID), ('LQR', C_LQR)]:
        r = results_balance[f'{cname}_{ang}']
        t = time_ax(len(r['traj']))
        st_str = f"{r['settling_ms']:.0f} ms" if r['settling_ms'] else "no settle"
        ax.plot(t, np.degrees(r['traj'][:, 2]), color=col, lw=1.6,
                label=f"{cname}  ({st_str})")
    ax.axhline(0,  color='k',    lw=0.7, ls=':')
    ax.axhline( 2, color='grey', lw=0.8, ls='--', alpha=0.5)
    ax.axhline(-2, color='grey', lw=0.8, ls='--', alpha=0.5)
    ax.set_title(f'Initial angle: {ang:.0f}°', fontsize=10)
    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_xlim(0, len(r['traj']) * DT)
    ax.grid(True, alpha=0.3, ls='--')
    ax.legend(fontsize=8)

axes[0].set_ylabel('Pendulum Angle (°)', fontsize=9)
plt.tight_layout()
plt.savefig('plot_eval_b.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_eval_b.png")


# ============================================================================
# PLOT 3 — Eval C: sprint position + angle
# ============================================================================
fig, axes = plt.subplots(2, 2, figsize=(13, 8))
fig.suptitle('Simulation: Evaluation C — Sprint & Stop',
             fontsize=13, fontweight='bold')

for col_idx, (cname, col, full) in enumerate([
        ('TPID', C_TPID, 'TrajectoryPID'),
        ('LQR',  C_LQR,  'LQR')]):
    r    = results_sprint[cname]
    traj = r['traj']
    t    = time_ax(len(traj))

    ax_x = axes[0, col_idx]
    ax_t = axes[1, col_idx]

    # Position
    ax_x.plot(t, traj[:, 0], color=col, lw=1.8)
    ax_x.axhline(2.0, color='red', ls='--', lw=1.2, label='Target 2.0 m')
    ax_x.axhspan(1.95, 2.05, color='red', alpha=0.1, label='±5 cm tolerance')
    ax_x.set_title(f'{full} — Cart Position', fontsize=10, fontweight='bold')
    ax_x.set_ylabel('Position (m)', fontsize=9)
    ax_x.set_xlabel('Time (s)', fontsize=9)
    ax_x.legend(fontsize=8)
    ax_x.grid(True, alpha=0.3, ls='--')
    ax_x.set_xlim(0, len(traj) * DT)

    # Angle
    result_str = (f"Arrival: {r['arrival_s']:.2f} s"
                  if r['arrival_s'] else "FAIL — did not arrive")
    ax_t.plot(t, np.degrees(traj[:, 2]), color=col, lw=1.8)
    ax_t.axhline(0,  color='k',    lw=0.7, ls=':')
    ax_t.axhline( 5, color='grey', lw=0.8, ls='--', alpha=0.5, label='±5° stability band')
    ax_t.axhline(-5, color='grey', lw=0.8, ls='--', alpha=0.5)
    ax_t.set_title(f'{full} — Angle  [{result_str}]', fontsize=10, fontweight='bold')
    ax_t.set_ylabel('Angle (°)', fontsize=9)
    ax_t.set_xlabel('Time (s)', fontsize=9)
    ax_t.legend(fontsize=8)
    ax_t.grid(True, alpha=0.3, ls='--')
    ax_t.set_xlim(0, len(traj) * DT)

plt.tight_layout()
plt.savefig('plot_eval_c.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_eval_c.png")


# ============================================================================
# PLOT 4 — Noise sensitivity
# ============================================================================
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
fig.suptitle('Simulation: Experiment 2 — Noise Sensitivity  (Eval B, 10° start)',
             fontsize=12, fontweight='bold')

for ax, (metric, ylabel) in zip(axes, [
        ('settling_ms',   'Settling Time (ms)'),
        ('rms_theta_deg', 'Post-settling RMS θ (°)')]):
    for cname, col, mk in [('PID', C_PID, 'o'), ('LQR', C_LQR, 's')]:
        vals = [r[metric] if r[metric] is not None else np.nan
                for r in results_noise[cname]]
        ax.plot(noise_levels, vals, color=col, marker=mk, lw=2, ms=7, label=cname)
    ax.axvline(0.01, color='grey', ls=':', lw=1.2, label='Hardware noise (σ=0.01)')
    ax.set_xlabel('Sensor Noise σ (rad)', fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(ylabel, fontsize=10, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, ls='--')

plt.tight_layout()
plt.savefig('plot_noise.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_noise.png")


# ============================================================================
# PLOT 5 — Filter window sensitivity
# ============================================================================
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
fig.suptitle('Simulation: Experiment 3 — Filter Window Size  (Eval B, 10°, σ=0.01)',
             fontsize=12, fontweight='bold')

for ax, (metric, ylabel) in zip(axes, [
        ('settling_ms', 'Settling Time (ms)'),
        ('rms_ctrl',    'RMS Control Effort (N)')]):
    for cname, col, mk in [('PID', C_PID, 'o'), ('LQR', C_LQR, 's')]:
        vals = [r[metric] if r[metric] is not None else np.nan
                for r in results_window[cname]]
        ax.plot(window_sizes, vals, color=col, marker=mk, lw=2, ms=7, label=cname)
    ax.axvline(50, color='grey', ls=':', lw=1.2, label='Hardware window (50)')
    ax.set_xlabel('Filter Window Size', fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(ylabel, fontsize=10, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, ls='--')

plt.tight_layout()
plt.savefig('plot_window.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_window.png")


# ============================================================================
# PLOT 6 — LQR Q[2,2] sensitivity
# ============================================================================
q_vals    = [r['Q22']     for r in results_q]
k_thetas  = [r['K_theta'] for r in results_q]
settles   = [r['settling_ms'] for r in results_q]
rms_ctrls = [r['rms_ctrl']    for r in results_q]

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
fig.suptitle('Simulation: Experiment 4 — LQR Q[2,2] Angle Penalty Sensitivity'
             '\n(Eval B, 10°, σ=0.01)', fontsize=12, fontweight='bold')

for ax, ydata, ylabel, marker in [
        (axes[0], k_thetas,  '$k_\\theta$ vs Q[2,2]',        'o'),
        (axes[1], settles,   'Settling Time (ms)',            's'),
        (axes[2], rms_ctrls, 'RMS Control Effort (N)',        '^')]:
    ax.plot(q_vals, ydata, color=C_LQR, marker=marker, lw=2, ms=7)
    ax.axvline(100, color='grey', ls=':', lw=1.2, label='Hardware Q[2,2]=100')
    ax.set_xlabel('Q[2,2]', fontsize=10)
    ax.set_ylabel(ylabel,   fontsize=10)
    ax.set_title(ylabel,    fontsize=10, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, ls='--')

plt.tight_layout()
plt.savefig('plot_lqr_q.png', dpi=160, bbox_inches='tight')
plt.close()
print("Saved plot_lqr_q.png")

print("\nAll 6 plots saved successfully.")
