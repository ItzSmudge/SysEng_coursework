"""
demo.py  —  Inverted Pendulum Interactive Demo
===============================================

Usage
-----
    python demo.py --eval A --controller lqr
    python demo.py --eval B --controller lqr --angle 10
    python demo.py --eval C --controller lqr

Arguments
---------
  --eval        A | B | C          Evaluation scenario (default: A)
  --controller  pid | lqr          Controller type    (default: lqr)
  --angle       6.5 | 10 | 15      V-block start angle for Eval B (default: 6.5)
  --steps       int                Simulation steps   (default: 10000)
  --noise       float              Sensor noise std   (default: 0.01)

Two windows open:
  Window 1 — Animation + all controls (gains, disturbances, buttons)
  Window 2 — Live dynamic plots (angle, position, velocities, control input)

Disturbances
------------
  "Tap Pendulum" button  — applies an impulse to theta_dot (angular)
  "Shove Cart"   button  — applies an impulse to x_dot     (linear)
  Each has its own magnitude slider.
"""

import argparse
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib
import os as _os
if not _os.environ.get('MPLBACKEND'):
    for _b in ['TkAgg','Qt5Agg','Qt6Agg','wxAgg','macosx']:
        try:
            matplotlib.use(_b); break
        except Exception: continue
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib.widgets import Slider, Button
import control as ct

from pendulum import Pendulum
from controllers import PIDController, LQRController, TrajectoryPIDController
from filters import MovingAverageFilter

# ── Physical parameters (hardware-measured) ──────────────────────────────────
M_CART = 1.3816
M_BOB  = 0.05
L_ROD  = 0.6
B_FRIC = 0.1
DT     = 0.001

# ── Default gains ─────────────────────────────────────────────────────────────
Q_LQR  = np.diag([1.0, 1.0, 100.0, 10.0])
R_LQR  = np.array([[0.1]])

PID_GAINS = dict(kp_theta=120.0, kd_theta=20.0, ki_theta=5.0,
                 kp_x=0.0,       kd_x=0.0,      ki_x=0.0)

# ── Eval B jerk parameters (calibrated against simulation) ───────────────────
# Jerk direction is negative (cart moves away from lean direction so pendulum
# swings up through vertical). Force in Newtons (simulation units).
JERK_FORCE    = -5.0           # N  — sufficient to swing up from all V-block angles
JERK_DURATION = {6.5:  0.200,  # seconds of kick per start angle
                 10.0: 0.200,
                 15.0: 0.200}
COAST_DURATION    = 0.040      # seconds of zero force after kick (pendulum coasts)
CAPTURE_THRESHOLD = 0.35       # rad — LQR takes over when |θ| < this

# ── Colours ───────────────────────────────────────────────────────────────────
C_CART   = '#2E75B6'
C_PEND   = '#C00000'
C_TRACE  = '#70AD47'
C_ANGLE  = '#ED7D31'
C_POS    = '#4472C4'
C_VEL    = '#70AD47'
C_ANGVEL = '#ED7D31'
C_CTRL   = '#7030A0'
C_TAP    = '#C00000'
C_SHOVE  = '#2E75B6'


# ══════════════════════════════════════════════════════════════════════════════
# Simulation state machine
# ══════════════════════════════════════════════════════════════════════════════
class SimState:
    IDLE      = 'IDLE'
    JERK      = 'JERK'
    COAST     = 'COAST'
    BALANCING = 'BALANCING'
    FALLEN    = 'FALLEN'


class Demo:
    def __init__(self, eval_id, controller_type, start_angle, steps, noise_std, use_jerk=False):
        self.eval_id         = eval_id.upper()
        self.controller_type = controller_type.lower()
        self.start_angle_deg = float(start_angle)
        self.steps           = steps
        self.noise_std       = noise_std
        self.use_jerk        = use_jerk

        # Build pendulum
        self.pend = Pendulum(
            M=M_CART, m=M_BOB, l=L_ROD, b=B_FRIC,
            dt=DT, mode="0", disturbance_level=0,
            noise_std_dev=noise_std
        )

        # Build controller
        self._build_controller()

        # Initial state
        if self.eval_id == 'B':
            ang = np.radians(self.start_angle_deg)
        else:
            ang = 0.0
        self.initial_state = np.array([0.0, 0.0, ang, 0.0])

        # Sprint target
        self.target_pos = [2.0, 0.0] if self.eval_id == 'C' else [0.0, 0.0]

        # Ring-buffer for live plots (keep last `steps` samples)
        self.buf_len    = steps
        self.buf_t      = np.zeros(self.buf_len)
        self.buf_x      = np.zeros(self.buf_len)
        self.buf_theta  = np.zeros(self.buf_len)
        self.buf_xdot   = np.zeros(self.buf_len)
        self.buf_tdot   = np.zeros(self.buf_len)
        self.buf_ctrl   = np.zeros(self.buf_len)
        self.buf_idx    = 0          # write pointer
        self.buf_count  = 0          # total samples written

        # Simulation live state
        self.state       = self.initial_state.copy()
        self.sim_time    = 0.0
        self.sim_step    = 0
        self.paused      = True
        self.machine     = SimState.IDLE
        self.jerk_timer  = 0.0
        self.coast_timer = 0.0

        # Pending disturbance (set by button callbacks)
        self._pending_Fd = None

        # Animation trace
        self.trace_x = []
        self.trace_y = []

        self._build_ui()

    # ── Controller factory ────────────────────────────────────────────────────
    def _build_controller(self):
        if self.controller_type == 'pid':
            self.controller = PIDController(
                **PID_GAINS, dt=DT,
                filter_enabled=True, window_size=5
            )
        elif self.controller_type == 'lqr':
            self.controller = LQRController(
                M=M_CART, m=M_BOB, l=L_ROD, b=B_FRIC,
                Q=Q_LQR, R=R_LQR,
                filter_enabled=True, window_size=5
            )
        else:
            raise ValueError(f"Unknown controller: {self.controller_type}")

    # ── Reset ─────────────────────────────────────────────────────────────────
    def _reset(self):
        if self.eval_id == 'B':
            ang = np.radians(self.start_angle_deg)
        else:
            ang = 0.0
        self.state       = np.array([0.0, 0.0, ang, 0.0])
        self.sim_time    = 0.0
        self.sim_step    = 0
        self.paused      = True
        self.machine     = SimState.IDLE
        self.jerk_timer  = 0.0
        self.coast_timer = 0.0
        self._pending_Fd = None
        self.trace_x.clear()
        self.trace_y.clear()

        # Clear buffers
        self.buf_t[:]     = 0
        self.buf_x[:]     = 0
        self.buf_theta[:] = 0
        self.buf_xdot[:]  = 0
        self.buf_tdot[:]  = 0
        self.buf_ctrl[:]  = 0
        self.buf_idx   = 0
        self.buf_count = 0

        self.controller.reset()
        self._rebuild_lqr_if_needed()

        # Reset UI
        self._update_status(f"RESET — Eval {self.eval_id} | {self.controller_type.upper()} | Press ▶ Play")

    # ── Recompute LQR gains from sliders ─────────────────────────────────────
    def _rebuild_lqr_if_needed(self):
        if self.controller_type != 'lqr':
            return
        Q = np.diag([
            self.sl_q_x.val,
            self.sl_q_xdot.val,
            self.sl_q_theta.val,
            self.sl_q_tdot.val,
        ])
        R = np.array([[self.sl_r.val]])
        I_val = (1/3) * M_BOB * L_ROD**2
        denom = I_val*(M_CART+M_BOB) + M_CART*M_BOB*L_ROD**2
        A = np.array([
            [0, 1, 0, 0],
            [0, -(I_val+M_BOB*L_ROD**2)*B_FRIC/denom,
               (M_BOB**2*L_ROD**2*9.81)/denom, 0],
            [0, 0, 0, 1],
            [0, -(M_BOB*L_ROD*B_FRIC)/denom,
               (M_BOB*L_ROD*9.81*(M_CART+M_BOB))/denom, 0]
        ])
        B = np.array([[0],
                      [(I_val+M_BOB*L_ROD**2)/denom],
                      [0],
                      [(M_BOB*L_ROD)/denom]])
        self.controller.K, _, _ = ct.lqr(A, B, Q, R)
        self.controller.Q = Q
        self.controller.R = R

    # ══════════════════════════════════════════════════════════════════════════
    # UI BUILD
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        # ── Window 1: Animation + Controls ───────────────────────────────────
        self.fig_anim = plt.figure(figsize=(14, 9))
        jerk_str = ' | JERK' if (self.eval_id == 'B' and self.use_jerk) else ''
        self.fig_anim.canvas.manager.set_window_title(
            f'Pendulum Demo — Eval {self.eval_id}{jerk_str} | {self.controller_type.upper()}')
        self.fig_anim.patch.set_facecolor('#1C1C1C')

        # Animation axis: top 65% of the figure, right 75%
        self.ax_anim = self.fig_anim.add_axes([0.23, 0.30, 0.74, 0.63])
        self.ax_anim.set_facecolor('#111111')
        self.ax_anim.set_xlim(-3.2, 3.2)
        self.ax_anim.set_ylim(-0.8, 1.5)
        self.ax_anim.set_aspect('equal')
        self.ax_anim.grid(True, color='#333333', linewidth=0.5)
        self.ax_anim.set_xlabel('Position (m)', color='white', fontsize=9)
        self.ax_anim.set_ylabel('Height (m)', color='white', fontsize=9)
        self.ax_anim.tick_params(colors='white', labelsize=8)
        for sp in self.ax_anim.spines.values():
            sp.set_color('#444444')

        # Target line for Eval C
        if self.eval_id == 'C':
            self.ax_anim.axvline(2.0, color='#FFD700', lw=1.2, ls='--', alpha=0.6)
            self.ax_anim.text(2.05, 1.3, '2 m target', color='#FFD700',
                              fontsize=8, va='top')

        # Ground line
        self.ax_anim.axhline(0, color='#555555', lw=1.0)

        # Cart and pendulum visuals
        cw, ch = 0.30, 0.12
        self.cart_patch = Rectangle((-cw/2, -ch), cw, ch,
                                    facecolor=C_CART, edgecolor='white',
                                    linewidth=1.0, zorder=3)
        self.ax_anim.add_patch(self.cart_patch)
        self.pend_line, = self.ax_anim.plot([], [], '-o',
                                             color=C_PEND, lw=2.5,
                                             markersize=10, zorder=4)
        self.trace_line, = self.ax_anim.plot([], [], '-',
                                              color=C_TRACE, lw=0.8,
                                              alpha=0.5, zorder=2)

        # Disturbance arrow
        self.dist_arrow = self.ax_anim.annotate(
            '', xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='yellow',
                            lw=2.5, mutation_scale=18),
            zorder=5, alpha=0.0)
        self._dist_arrow_timer = 0

        # Status / info text
        self.status_text = self.ax_anim.text(
            0.01, 0.99, '', transform=self.ax_anim.transAxes,
            color='white', fontsize=9, va='top', ha='left',
            bbox=dict(facecolor='#222222', edgecolor='none',
                      boxstyle='round,pad=0.3', alpha=0.8), zorder=6)
        self._update_status(f"Eval {self.eval_id} | {self.controller_type.upper()} | Press ▶ Play")

        # ── Left panel: controller gain sliders ──────────────────────────────
        self._build_gain_sliders()

        # ── Bottom panel: buttons + disturbance controls ──────────────────────
        self._build_bottom_controls()

        # ── Window 2: Live plots ──────────────────────────────────────────────
        self.fig_plots = plt.figure(figsize=(12, 8))
        self.fig_plots.canvas.manager.set_window_title('Live State Plots')
        self.fig_plots.patch.set_facecolor('#1C1C1C')

        gs = gridspec.GridSpec(2, 2, figure=self.fig_plots,
                               hspace=0.45, wspace=0.35,
                               left=0.09, right=0.97,
                               top=0.93, bottom=0.09)

        plot_kw = dict(facecolor='#111111')
        tick_kw = dict(colors='white', labelsize=8)

        self.ax_angle   = self.fig_plots.add_subplot(gs[0, 0], **plot_kw)
        self.ax_pos     = self.fig_plots.add_subplot(gs[0, 1], **plot_kw)
        self.ax_vel     = self.fig_plots.add_subplot(gs[1, 0], **plot_kw)
        self.ax_ctrl    = self.fig_plots.add_subplot(gs[1, 1], **plot_kw)

        for ax in [self.ax_angle, self.ax_pos, self.ax_vel, self.ax_ctrl]:
            ax.set_facecolor('#111111')
            ax.tick_params(**tick_kw)
            ax.grid(True, color='#333333', linewidth=0.5, linestyle='--')
            for sp in ax.spines.values():
                sp.set_color('#444444')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')

        # Live lines
        t0 = np.zeros(1)
        self.ln_angle,  = self.ax_angle.plot(t0, t0, color=C_ANGLE,  lw=1.4)
        self.ln_pos,    = self.ax_pos.plot(  t0, t0, color=C_POS,    lw=1.4)
        self.ln_xdot,   = self.ax_vel.plot(  t0, t0, color=C_VEL,    lw=1.4, label='ẋ')
        self.ln_tdot,   = self.ax_vel.plot(  t0, t0, color=C_ANGVEL,  lw=1.4, label='θ̇')
        self.ln_ctrl,   = self.ax_ctrl.plot( t0, t0, color=C_CTRL,   lw=1.4)

        # Reference lines
        self.ax_angle.axhline(0,  color='white', lw=0.6, ls=':', alpha=0.4)
        self.ax_angle.axhline(2,  color='grey',  lw=0.6, ls='--', alpha=0.4)
        self.ax_angle.axhline(-2, color='grey',  lw=0.6, ls='--', alpha=0.4)
        self.ax_ctrl.axhline(0,   color='white', lw=0.6, ls=':', alpha=0.4)
        if self.eval_id == 'C':
            self.ax_pos.axhline(2.0, color='#FFD700', lw=0.8, ls='--', alpha=0.6)

        self.ax_angle.set_title('Pendulum Angle', fontsize=10, fontweight='bold')
        self.ax_pos.set_title('Cart Position',    fontsize=10, fontweight='bold')
        self.ax_vel.set_title('Velocities',       fontsize=10, fontweight='bold')
        self.ax_ctrl.set_title('Control Input',   fontsize=10, fontweight='bold')

        self.ax_angle.set_xlabel('Time (s)', fontsize=8)
        self.ax_angle.set_ylabel('Angle (°)', fontsize=8)
        self.ax_pos.set_xlabel('Time (s)', fontsize=8)
        self.ax_pos.set_ylabel('Position (m)', fontsize=8)
        self.ax_vel.set_xlabel('Time (s)', fontsize=8)
        self.ax_vel.set_ylabel('Velocity (m/s or rad/s)', fontsize=8)
        self.ax_ctrl.set_xlabel('Time (s)', fontsize=8)
        self.ax_ctrl.set_ylabel('Force (N)', fontsize=8)

        self.ax_vel.legend(fontsize=8, facecolor='#222222',
                           labelcolor='white', loc='upper right')

        self.fig_plots.suptitle(
            f'Live Plots — Eval {self.eval_id} | {self.controller_type.upper()}',
            color='white', fontsize=11, fontweight='bold')

    # ── Gain sliders (left panel of Window 1) ────────────────────────────────
    def _build_gain_sliders(self):
        sl_kw = dict(color='#333333')
        lbl_kw = dict(color='white', fontsize=8)

        # Header
        self.fig_anim.text(0.005, 0.97, 'Controller Gains',
                           color='white', fontsize=9, fontweight='bold',
                           transform=self.fig_anim.transFigure)

        if self.controller_type == 'lqr':
            configs = [
                ('sl_q_x',     'Q[x]',     0.1, 500,  Q_LQR[0,0]),
                ('sl_q_xdot',  'Q[ẋ]',     0.1, 100,  Q_LQR[1,1]),
                ('sl_q_theta', 'Q[θ]',     0.1, 500,  Q_LQR[2,2]),
                ('sl_q_tdot',  'Q[θ̇]',    0.1, 200,  Q_LQR[3,3]),
                ('sl_r',       'R',         0.001, 5, R_LQR[0,0]),
            ]
        else:
            configs = [
                ('sl_kp_theta', 'Kp θ',    0, 300,  120.0),
                ('sl_kd_theta', 'Kd θ',    0, 80,   20.0),
                ('sl_ki_theta', 'Ki θ',    0, 20,   5.0),
                ('sl_kp_x',    'Kp x',     0, 50,   0.0),
                ('sl_kd_x',    'Kd x',     0, 50,   0.0),
                ('sl_ki_x',    'Ki x',     0, 10,   0.0),
            ]

        y_start = 0.90
        dy      = 0.055
        for i, (attr, label, vmin, vmax, vinit) in enumerate(configs):
            ax = self.fig_anim.add_axes([0.01, y_start - i*dy, 0.18, 0.025])
            sl = Slider(ax, label, vmin, vmax, valinit=vinit,
                        color='#ED7D31', track_color='#444444')
            sl.label.set_color('white')
            sl.label.set_fontsize(8)
            sl.valtext.set_color('white')
            sl.valtext.set_fontsize(7)
            setattr(self, attr, sl)

    # ── Bottom controls (buttons + disturbance sliders) ───────────────────────
    def _build_bottom_controls(self):
        fig = self.fig_anim

        # ── Row 1: play/pause/reset/apply ────────────────────────────────────
        btn_kw = dict(color='#333333', hovercolor='#555555')
        bw, bh, by = 0.10, 0.045, 0.195

        ax_play  = fig.add_axes([0.24, by, bw, bh])
        ax_pause = fig.add_axes([0.36, by, bw, bh])
        ax_reset = fig.add_axes([0.48, by, bw, bh])
        ax_apply = fig.add_axes([0.60, by, bw, bh])

        self.btn_play  = Button(ax_play,  '> Play',  **btn_kw)
        self.btn_pause = Button(ax_pause, '|| Pause', **btn_kw)
        self.btn_reset = Button(ax_reset, '<> Reset', **btn_kw)
        self.btn_apply = Button(ax_apply, '* Apply', **btn_kw)

        for btn in [self.btn_play, self.btn_pause,
                    self.btn_reset, self.btn_apply]:
            btn.label.set_color('white')
            btn.label.set_fontsize(9)

        self.btn_play.on_clicked(self._on_play)
        self.btn_pause.on_clicked(self._on_pause)
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_apply.on_clicked(self._on_apply)

        # ── Row 2: disturbance controls ───────────────────────────────────────
        # Pendulum tap
        fig.text(0.24, 0.135, 'Pendulum Tap',
                 color=C_TAP, fontsize=8, fontweight='bold')
        ax_tap_sl = fig.add_axes([0.24, 0.095, 0.20, 0.022])
        self.sl_tap_mag = Slider(ax_tap_sl, 'Mag', 0.1, 10.0, valinit=3.0,
                                 color=C_TAP, track_color='#444444')
        self.sl_tap_mag.label.set_color('white')
        self.sl_tap_mag.label.set_fontsize(8)
        self.sl_tap_mag.valtext.set_color('white')
        self.sl_tap_mag.valtext.set_fontsize(7)

        ax_tap_btn = fig.add_axes([0.24, 0.050, 0.20, 0.038])
        self.btn_tap = Button(ax_tap_btn, '[TAP] Pendulum',
                              color='#6B0000', hovercolor='#A00000')
        self.btn_tap.label.set_color('white')
        self.btn_tap.label.set_fontsize(9)
        self.btn_tap.on_clicked(self._on_tap)

        # Cart shove
        fig.text(0.52, 0.135, 'Cart Shove',
                 color=C_SHOVE, fontsize=8, fontweight='bold')
        ax_shove_sl = fig.add_axes([0.52, 0.095, 0.20, 0.022])
        self.sl_shove_mag = Slider(ax_shove_sl, 'Mag', 0.1, 10.0, valinit=3.0,
                                   color=C_SHOVE, track_color='#444444')
        self.sl_shove_mag.label.set_color('white')
        self.sl_shove_mag.label.set_fontsize(8)
        self.sl_shove_mag.valtext.set_color('white')
        self.sl_shove_mag.valtext.set_fontsize(7)

        ax_shove_btn = fig.add_axes([0.52, 0.050, 0.20, 0.038])
        self.btn_shove = Button(ax_shove_btn, '[SHOVE] Cart',
                                color='#003A6B', hovercolor='#005A9E')
        self.btn_shove.label.set_color('white')
        self.btn_shove.label.set_fontsize(9)
        self.btn_shove.on_clicked(self._on_shove)

        # Noise slider
        fig.text(0.76, 0.135, 'Sensor Noise',
                 color='#AAAAAA', fontsize=8, fontweight='bold')
        ax_noise_sl = fig.add_axes([0.76, 0.095, 0.20, 0.022])
        self.sl_noise = Slider(ax_noise_sl, 'σ', 0.0, 0.1,
                               valinit=self.noise_std,
                               color='#888888', track_color='#444444')
        self.sl_noise.label.set_color('white')
        self.sl_noise.label.set_fontsize(8)
        self.sl_noise.valtext.set_color('white')
        self.sl_noise.valtext.set_fontsize(7)

    # ── Status text ───────────────────────────────────────────────────────────
    def _update_status(self, msg):
        self.status_text.set_text(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # Button callbacks
    # ══════════════════════════════════════════════════════════════════════════
    def _on_play(self, event):
        if self.machine == SimState.IDLE and self.eval_id == 'B' and self.use_jerk:
            self.machine = SimState.JERK
            self.jerk_timer = 0.0
            self._update_status('JERK phase — swinging up...')
        elif self.machine == SimState.IDLE:
            # No jerk: drop straight into balancing (controller handles it from angle)
            self.machine = SimState.BALANCING
            self._update_status(f'BALANCING — Eval {self.eval_id}')
        self.paused = False

    def _on_pause(self, event):
        self.paused = True
        self._update_status('PAUSED')

    def _on_reset(self, event):
        self._reset()
        self._update_all_plots()
        self.fig_anim.canvas.draw_idle()
        self.fig_plots.canvas.draw_idle()

    def _on_apply(self, event):
        self.paused = True
        self._rebuild_lqr_if_needed()
        if self.controller_type == 'pid':
            self.controller.kp_theta = self.sl_kp_theta.val
            self.controller.kd_theta = self.sl_kd_theta.val
            self.controller.ki_theta = self.sl_ki_theta.val
            self.controller.kp_x    = self.sl_kp_x.val
            self.controller.kd_x    = self.sl_kd_x.val
            self.controller.ki_x    = self.sl_ki_x.val
        self.pend.noise_std_dev = self.sl_noise.val
        self._update_status('Gains applied — press ▶ Play to continue')

    def _on_tap(self, event):
        """Queue a pendulum angular impulse (theta_dot)."""
        mag = self.sl_tap_mag.val * 1000.0   # scale to simulation units
        # Random direction, or always positive — use random for realism
        direction = np.random.choice([-1, 1])
        self._pending_Fd = np.array([[0.0, direction * mag]])
        self._dist_arrow_timer = 40
        self._update_status(f'TAP applied: {direction*mag/1000:.1f} units (angular)')

    def _on_shove(self, event):
        """Queue a cart linear impulse (x_dot)."""
        mag = self.sl_shove_mag.val * 1000.0
        direction = np.random.choice([-1, 1])
        self._pending_Fd = np.array([[direction * mag, 0.0]])
        self._dist_arrow_timer = 40
        self._update_status(f'SHOVE applied: {direction*mag/1000:.1f} units (linear)')

    # ══════════════════════════════════════════════════════════════════════════
    # One simulation step
    # ══════════════════════════════════════════════════════════════════════════
    def _sim_step(self):
        """Advance the simulation by one DT, honouring the state machine."""
        state = self.state
        Fd = np.array([[0.0, 0.0]])

        # Consume any pending disturbance
        if self._pending_Fd is not None:
            Fd = self._pending_Fd
            self._pending_Fd = None

        # ── State machine ─────────────────────────────────────────────────────
        if self.machine == SimState.JERK:
            u = JERK_FORCE
            self.jerk_timer += DT
            jerk_dur = JERK_DURATION.get(self.start_angle_deg, 0.200)
            if self.jerk_timer >= jerk_dur:
                self.machine = SimState.COAST
                self.coast_timer = 0.0

        elif self.machine == SimState.COAST:
            u = 0.0
            self.coast_timer += DT
            theta = state[2]
            if self.coast_timer >= COAST_DURATION:
                if abs(theta) < CAPTURE_THRESHOLD:
                    self.machine = SimState.BALANCING
                    self._update_status('LQR captured — BALANCING')
                else:
                    self.machine = SimState.JERK
                    self.jerk_timer = 0.0
                    self._update_status('Missed — retrying JERK')

        elif self.machine == SimState.BALANCING:
            noisy = self.pend.get_noisy_observation(state)
            u = self.controller.get_action(noisy, self.target_pos)
            theta = state[2]
            if abs(theta) > np.radians(35):
                self.machine = SimState.FALLEN
                self.paused  = True
                self._update_status('FALLEN — press ↺ Reset')

        else:  # IDLE or FALLEN
            u = 0.0

        # ── Physics step ──────────────────────────────────────────────────────
        new_state = self.pend.step(state, u, Fd)
        self.state    = new_state
        self.sim_time += DT
        self.sim_step += 1

        # ── Write to ring buffer ──────────────────────────────────────────────
        i = self.buf_idx
        self.buf_t[i]     = self.sim_time
        self.buf_x[i]     = new_state[0]
        self.buf_theta[i] = np.degrees(new_state[2])
        self.buf_xdot[i]  = new_state[1]
        self.buf_tdot[i]  = np.degrees(new_state[3])
        self.buf_ctrl[i]  = u
        self.buf_idx   = (i + 1) % self.buf_len
        self.buf_count += 1

        return new_state, u, Fd

    def _get_buffer_ordered(self):
        """Return ordered (time, data) arrays from ring buffer."""
        n = min(self.buf_count, self.buf_len)
        if self.buf_count < self.buf_len:
            sl = slice(0, n)
        else:
            # Wrap: oldest is at buf_idx
            idx = self.buf_idx
            order = np.concatenate([np.arange(idx, self.buf_len),
                                    np.arange(0, idx)])
            return (self.buf_t[order],
                    self.buf_x[order],
                    self.buf_theta[order],
                    self.buf_xdot[order],
                    self.buf_tdot[order],
                    self.buf_ctrl[order])
        return (self.buf_t[:n], self.buf_x[:n], self.buf_theta[:n],
                self.buf_xdot[:n], self.buf_tdot[:n], self.buf_ctrl[:n])

    # ══════════════════════════════════════════════════════════════════════════
    # Plot updates
    # ══════════════════════════════════════════════════════════════════════════
    def _update_animation(self, state, u, Fd):
        x     = state[0]
        theta = state[2]
        cw, ch = 0.30, 0.12

        # Cart
        self.cart_patch.set_xy((x - cw/2, -ch))

        # Pendulum bob position
        px = x + L_ROD * np.sin(-theta)
        py = L_ROD * np.cos(-theta)
        self.pend_line.set_data([x, px], [0, py])

        # Trace
        self.trace_x.append(px)
        self.trace_y.append(py)
        if len(self.trace_x) > 800:
            self.trace_x.pop(0)
            self.trace_y.pop(0)
        self.trace_line.set_data(self.trace_x, self.trace_y)

        # Disturbance arrow
        if self._dist_arrow_timer > 0:
            self._dist_arrow_timer -= 1
            alpha = self._dist_arrow_timer / 40.0
            # Determine direction from Fd
            fx, ft = Fd[0, 0], Fd[0, 1]
            if abs(fx) > abs(ft):     # horizontal shove
                dx, dy = np.sign(fx) * 0.5, 0.0
                ox, oy = x, 0.05
            else:                     # pendulum tap
                dx, dy = 0.0, np.sign(ft) * 0.4
                ox, oy = px, py
            self.dist_arrow.set_position((ox + dx, oy + dy))
            self.dist_arrow.xy = (ox, oy)
            self.dist_arrow.set_alpha(alpha)
        else:
            self.dist_arrow.set_alpha(0.0)

        # Status machine label
        if not self.paused:
            t_str = f't={self.sim_time:.2f}s'
            th_str = f'θ={np.degrees(theta):.1f}°'
            self._update_status(
                f'{self.machine}  {t_str}  {th_str}  '
                f'u={u:.1f} N'
            )

    def _update_live_plots(self):
        data = self._get_buffer_ordered()
        t, x, theta, xdot, tdot, ctrl = data

        self.ln_angle.set_data(t, theta)
        self.ln_pos.set_data(t, x)
        self.ln_xdot.set_data(t, xdot)
        self.ln_tdot.set_data(t, tdot)
        self.ln_ctrl.set_data(t, ctrl)

        for ax in [self.ax_angle, self.ax_pos, self.ax_vel, self.ax_ctrl]:
            ax.relim()
            ax.autoscale_view()

    def _update_all_plots(self):
        self._update_animation(self.state, 0.0, np.array([[0.0, 0.0]]))
        self._update_live_plots()

    # ══════════════════════════════════════════════════════════════════════════
    # Animation driver
    # ══════════════════════════════════════════════════════════════════════════
    def animate(self, _frame):
        # Run multiple physics steps per frame for speed
        STEPS_PER_FRAME = 5
        last_state = self.state
        last_u     = 0.0
        last_Fd    = np.array([[0.0, 0.0]])

        if not self.paused:
            for _ in range(STEPS_PER_FRAME):
                last_state, last_u, last_Fd = self._sim_step()

        self._update_animation(last_state, last_u, last_Fd)

        # Update live plots every 10 frames to avoid lag
        if not hasattr(self, '_frame_counter'):
            self._frame_counter = 0
        self._frame_counter += 1
        if self._frame_counter % 10 == 0:
            self._update_live_plots()
            self.fig_plots.canvas.draw_idle()

        return []   # blit not used — full redraw

    # ══════════════════════════════════════════════════════════════════════════
    # Run
    # ══════════════════════════════════════════════════════════════════════════
    def run(self):
        import matplotlib.animation as anim_mod
        self._anim = anim_mod.FuncAnimation(
            self.fig_anim, self.animate,
            interval=16,          # ~60 fps
            blit=False,
            cache_frame_data=False
        )
        plt.show()


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='Inverted Pendulum Demo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    parser.add_argument('--eval',       default='A',
                        choices=['A', 'B', 'C'],
                        help='Evaluation scenario (default: A)')
    parser.add_argument('--controller', default='lqr',
                        choices=['pid', 'lqr'],
                        help='Controller type (default: lqr)')
    parser.add_argument('--angle',      default=6.5, type=float,
                        help='V-block start angle in degrees for Eval B (default: 6.5)')
    parser.add_argument('--steps',      default=10000, type=int,
                        help='Ring-buffer length in steps (default: 10000)')
    parser.add_argument('--noise',      default=0.01, type=float,
                        help='Sensor noise std dev (default: 0.01)')
    parser.add_argument('--jerk',       action='store_true',
                        help='Eval B: use open-loop jerk swing-up before handing to controller')

    args = parser.parse_args()

    print(f"\n  Inverted Pendulum Demo")
    print(f"  ─────────────────────────────────────")
    print(f"  Evaluation : {args.eval}")
    print(f"  Controller : {args.controller.upper()}")
    if args.eval == 'B':
        print(f"  Start angle: {args.angle}°")
        print(f"  Jerk mode  : {'ON (--jerk)' if args.jerk else 'OFF (direct balance)'}")
    print(f"  Noise σ    : {args.noise}")
    print(f"  ─────────────────────────────────────\n")
    print("  Press ▶ Play in the animation window to start.\n")

    demo = Demo(
        eval_id        = args.eval,
        controller_type= args.controller,
        start_angle    = args.angle,
        steps          = args.steps,
        noise_std      = args.noise,
        use_jerk       = args.jerk,
    )
    demo.run()


if __name__ == '__main__':
    main()
