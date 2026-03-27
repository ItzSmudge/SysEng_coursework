# Inverted Pendulum — COMP0216 Systems Engineering
**Team 3 · UCL MEng Robotics and AI · March 2026**

Aayan Islam · Xavier Parker · Peter Neville · Helitha Cooray

---

A Python simulation and control environment for a self-balancing inverted pendulum cart, built as part of the UCL COMP0216 Systems Engineering module. Includes a full physics simulation with RK4 integration, PID and LQR controllers, an interactive real-time demo, and the SysML model produced in Cameo Magic.

The physical system was built on an Arduino Giga R1 with four Pololu 25D motors, a Broadcom AS22 optical encoder, and two Motoron I²C motor drivers. The embedded code is also included.

---

## Table of Contents

- [Hardware](#hardware)
- [Repo structure](#repo-structure)
- [Getting started](#getting-started)
- [Running the demo](#running-the-demo)
- [SysML diagrams](#sysml-diagrams)
- [Arduino code](#arduino-code)

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | Arduino Giga R1 (480 MHz, 1 MB RAM) |
| Motors | 4× Pololu 25D metal gearmotors |
| Motor drivers | 2× Motoron M3S550 (I²C, addresses 16 and 17) |
| Pendulum encoder | Broadcom AS22 optical encoder (4096 CPR) |
| Position sensing | Motor encoders (232.32 CPR after gearing) |
| Chassis | 3 mm laser-cut acrylic |
| Supply voltage | 10.8 V (nominal 12.3 V) |
| Pendulum length | 0.60 m to centre of mass |
| Cart mass | 1.3816 kg |
| Bob mass | 50 g |

---

## Repo structure

```
.
├── simulation/
│   ├── pendulum.py       # Pendulum dynamics — 4th-order Runge–Kutta integrator
│   ├── controllers.py    # PIDController, LQRController, TrajectoryPIDController
│   ├── filters.py        # MovingAverageFilter
│   └── demo.py           # Interactive two-window visualisation with CLI flags
│
├── sysml/
│   └── team3_pendulum.mdzip   # Cameo Magic project file (all 6 SysML diagrams)
│
├── arduino/
│   └── pendulum_lqr/
│       └── pendulum_lqr.ino   # Embedded LQR + jerk swing-up + serial interface
│
├── requirements.txt
└── README.md
```

---

## Getting started

### Requirements

- Python 3.9 or later
- pip

Tested on Windows 11, Ubuntu 22.04, and macOS 14. No GPU required.

### Install

```bash
git clone https://github.com/ItzSmudge/SysEng_coursework.git
cd SysEng_coursework
pip install -r requirements.txt
```

If you have multiple Python versions installed:

```bash
python3 -m pip install -r requirements.txt
```

### Matplotlib backend

The demo opens two live windows and needs a display. On most desktop installs this works without any extra steps. If you hit a backend error (common on headless Linux), set the backend before running:

```bash
export MPLBACKEND=Qt5Agg   # or TkAgg, Qt6Agg, wxAgg depending on what's installed
```

---

## Running the demo

`demo.py` opens two windows simultaneously:

- **Window 1** — pendulum animation with gain sliders, disturbance buttons, and playback controls
- **Window 2** — live plots of pendulum angle, cart position, velocities, and control input, updating in real time as the simulation runs

### Flags

| Flag | Options | Default | Description |
|---|---|---|---|
| `--eval` | `A` `B` `C` | `A` | Evaluation scenario |
| `--controller` | `lqr` `pid` | `lqr` | Controller type |
| `--angle` | `6.5` `10` `15` | `6.5` | V-block start angle in degrees (Eval B only) |
| `--jerk` | — | off | Eval B: run open-loop jerk swing-up before handing to the controller |
| `--noise` | float | `0.01` | Sensor noise standard deviation in rad |
| `--steps` | int | `10000` | Ring-buffer length |

### Examples

```bash
# Eval A — LQR balancing from upright
python simulation/demo.py --eval A --controller lqr

# Eval A — PID
python simulation/demo.py --eval A --controller pid

# Eval B — start at 6.5°, controller balances directly from the angle (no jerk)
python simulation/demo.py --eval B --controller lqr --angle 6.5

# Eval B — start at 10°, open-loop jerk swing-up then LQR takes over
python simulation/demo.py --eval B --controller lqr --angle 10 --jerk

# Eval B — 15° with jerk, PID after capture
python simulation/demo.py --eval B --controller pid --angle 15 --jerk

# Eval C — sprint to 2 m while balancing
python simulation/demo.py --eval C --controller lqr
```

### Controls

**Playback buttons:**

| Button | What it does |
|---|---|
| `> Play` | Start or resume |
| `|| Pause` | Freeze at current frame |
| `<> Reset` | Return to initial conditions |
| `* Apply` | Apply slider changes (pauses first) |

**Disturbance buttons** (bottom of Window 1):

- `[TAP] Pendulum` — angular impulse on the pendulum (`theta_dot`). Direction is randomised. Magnitude controlled by the slider next to it.
- `[SHOVE] Cart` — linear impulse on the cart (`x_dot`). Has its own separate magnitude slider.

Both are injected correctly into the physics step — not scheduled through the old `simulate()` loop, which had a timing bug.

**Gain sliders** (left panel):

For LQR these adjust the Q matrix diagonal entries and R. For PID they adjust Kp, Kd, Ki for both the angle and position loops. Click `* Apply` to recompute and continue.

---

## SysML diagrams

All diagrams are in `sysml/team3_pendulum.mdzip`. Open this file in CATIA Magic to view and edit them.

The project contains six diagrams:

| Diagram | What it covers |
|---|---|
| Block Definition Diagram (BDD) | System decomposition into SensorSubsystem, ControllerSubsystem, ActuatorSubsystem |
| Internal Block Diagram (IBD) | Signal flow with ports — θ (rad), x (m), u (PWM), I²C addresses 16/17 |
| Use Case Diagram | Seven use cases across Operator, Sensor, Actuator, and Evaluator actors |
| Activity Diagram | Eval B swing-up control flow: GO → JERK → coast → monitor → LQR capture |
| State Machine Diagram | IDLE / JERK / BALANCING / FALLEN states with guard conditions and outputs |
| Requirements Diagram | REQ-01/02/03 and Constraints with deriveReqt and satisfy relationships |

If you do not have CATIA installed, PNG exports of all six diagrams are in the project report.

---

## Arduino code

`arduino/pendulum_lqr/pendulum_lqr.ino` is the embedded implementation that ran on the physical cart during the demo.

**To upload:** open in the Arduino IDE, select **Arduino Giga R1 WiFi** as the board, install the Motoron library via the Library Manager, then upload.

**How it works:**

- The Riccati equation is solved offline in Python and the resulting K vector is hardcoded. The Giga does not run Riccati at runtime.
- Sensor sampling runs at 10 kHz via `mbed::Ticker` interrupt. The control loop runs at 100 Hz.
- A 5-sample moving average filter is applied to both angle and position signals before the controller reads them.
- A +200 unit motor bias is added to all non-zero commands to overcome the physical deadband of the Motoron drivers.
- **Serial commands:** send `GO` to start, `STOP` to halt. Gains can be updated at runtime without recompiling — e.g. `K 700 2250` sets k_theta and k_theta_dot.
- **Eval B jerk sequence:** on `GO`, the cart runs at 750 raw units for 200 ms (6.5° case) or 230 ms (10° case), coasts for 40 ms, then switches to LQR when |θ| < 0.35 rad.

---

## Notes

The physical hardware was decommissioned on 26 March 2026 following the live demonstration on 24 March. Formal benchmarking data for the hardware was not collected — all quantitative results in the report are from the simulation (Section 5). Hardware observations from pre-demonstration tuning sessions are discussed in Section 6.

The full project planning artefacts — WBS, FMEA, risk matrix, Kanban board — are on the team Miro board linked in the report appendix.

---

## Licence

MIT — feel free to use or adapt this code. Please do not submit it as your own coursework.
