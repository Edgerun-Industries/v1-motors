# Simulation Design

## Background

The v1-motors firmware runs on an ESP32-S3 and communicates with a CubeMars motor
over CAN bus at 1 Mbps. Testing firmware changes, control algorithm behavior, or
command sequences requires a powered motor, a wired CAN bus, and a flashed board.
Simulation removes that dependency for the subset of problems that are software-only.

This document defines three simulation levels, what each one tests, how to build it,
and where its limits are.

---

## What simulation will NOT replace

Before describing what simulation covers, it is important to be clear about what
it cannot:

- **CAN bus timing and arbitration**: TWAI interrupt latency, message prioritization,
  and bus error recovery must be verified on hardware.
- **FreeRTOS task scheduling**: The interaction between `can_rx` (priority 1, core 1),
  `ctrl` (priority 4, core 0), and `serial` (priority 3, core 0) under real-time
  conditions cannot be replicated in Python.
- **Motor back-EMF, cogging, and thermal behavior**: These require physical measurement
  and datasheet values from CubeMars. Fabricated values produce misleading results.
- **USB serial latency**: The `usb_serial_jtag` driver behavior is hardware-specific.

---

## Level A — State Machine Simulator

### Purpose

Tests the command state machine in `motor_terminal.py` without any hardware.
The simulator speaks the exact same line protocol as the real ESP32 firmware
(ACK/NACK/STATUS strings), so `motor_terminal.py` requires zero modification.

### Architecture

```
motor_terminal.py --simulate
        |
        v
SimulatedMotor  (replaces serial port)
  - maintains motor_state_t equivalent in Python
  - parses commands: enable, disable, stop, spin_once, status, ping, id, scan_ids
  - produces ACK/NACK/STATUS lines in the same format as firmware logs
  - spin_once resolves immediately with SPIN_DONE
```

### State maintained

| Field     | Type  | Notes                                        |
| --------- | ----- | -------------------------------------------- |
| `enabled` | bool  | Set by `enable`, cleared by `stop`/`disable` |
| `fault`   | bool  | Can be injected via `/inject fault` command  |
| `fb_seen` | bool  | Cleared on `enable`, set on first tick       |
| `p_fb`    | float | Position feedback (rad)                      |
| `v_fb`    | float | Velocity feedback (rad/s)                    |
| `t_fb`    | float | Torque feedback (Nm)                         |

### Test scenarios enabled

- `enable → spin_once cw → status` — verify SPIN_DONE is reported
- `enable → spin_once cw → /inject fault → status` — verify fault state appears
- `enable` while `fault=true` — verify NACK is returned
- `disable` — verify fault is cleared, motor exits mode
- Scripted sequences: run the full command sequence unattended, assert expected responses
- CI without hardware: run in a GitHub Actions runner or local script

### Command extensions (simulator-only)

These commands are available only in `--simulate` mode:

```
/inject fault          — immediately set fault=true, trigger fb_timeout path
/inject no_feedback    — prevent fb_seen from being set (simulates CAN cable unplugged before enable)
/inject spin_timeout   — make next spin_once hit 8-second timeout instead of converging
```

### Implementation sketch

```python
class SimulatedMotor:
    def __init__(self) -> None:
        self.enabled: bool = False
        self.fault: bool = False
        self.fb_seen: bool = False
        self.p_fb: float = 0.0
        self.v_fb: float = 0.0
        self.t_fb: float = 0.0
        self._inject_fault: bool = False

    def send(self, cmd: str) -> list[str]:
        """Process one command line, return list of log lines to emit."""
        ...
```

The `send()` method mirrors `handle_command()` in `main.c` exactly, including the
NACK strings and STATUS field order, so the same `motor_terminal.py` parsing code
works unchanged.

---

## Level B — Physics Simulator

### Purpose

Adds a first-order motor dynamics model. `spin_once` takes real simulated time to
converge. Gain sensitivity can be explored without reflashing. The ramp trajectory
(Phase 3.2) can be validated offline before testing on hardware.

### Motor model

The CubeMars motor in MIT mode is commanded with a torque feedforward plus an
impedance term:

$$\tau_{cmd} = K_p \cdot (\theta_{target} - \theta) + K_d \cdot (0 - \dot{\theta}) + \tau_{ff}$$

The motor dynamics are modeled as a first-order system (rotor inertia + viscous damping):

$$J \ddot{\theta} = \tau_{cmd} - b \dot{\theta}$$

Integrated using the semi-implicit Euler method at the firmware control loop rate:

$$\dot{\theta}_{n+1} = \dot{\theta}_n + \frac{\Delta t}{J} \left( \tau_{cmd,n} - b \dot{\theta}_n \right)$$
$$\theta_{n+1} = \theta_n + \Delta t \cdot \dot{\theta}_{n+1}$$

Semi-implicit Euler is preferred over forward Euler because it is unconditionally
stable for this system class, which matters when the user is exploring high KP values.

### Parameters

| Parameter       | Symbol      | Default       | Notes                                  |
| --------------- | ----------- | ------------- | -------------------------------------- |
| Rotor inertia   | J           | 0.001 kg·m²   | AK60 range; adjust from datasheet      |
| Viscous damping | b           | 0.01 Nm·s/rad | Estimate; calibrate from step response |
| Control period  | Δt          | 0.005 s       | Matches `CONTROL_PERIOD_MS 5`          |
| Position limits | P_MIN/P_MAX | ±12.5 rad     | Matches firmware constants             |
| Torque limits   | T_MIN/T_MAX | ±32 Nm        | Matches firmware constants             |

These are initial estimates. Real calibration requires:

1. A step response test on hardware (command 1 Nm, measure velocity rise rate → gives J)
2. A coast-down test (spin at known velocity, cut torque, measure deceleration → gives b)

### Test scenarios enabled

- **Gain sweep**: run `spin_once cw` with KP=[5, 10, 20, 50, 100], plot settling time
  and overshoot. Verify KP=20 is not on the instability boundary.
- **SPIN_TIMEOUT reproduction**: set KP=0.01, verify 8-second timeout fires correctly
  and motor exits mode.
- **Ramp vs. step trajectory**: compare position tracking error for a direct step
  command vs. trapezoidal velocity ramp (Phase 3.2 planning).
- **Torque-only mode**: command constant torque (KP=0, KD=0), verify velocity
  increases correctly given J and b.

### Implementation sketch

```python
class PhysicsMotor(SimulatedMotor):
    def __init__(self, J: float = 0.001, b: float = 0.01, dt: float = 0.005) -> None:
        super().__init__()
        self.J = J
        self.b = b
        self.dt = dt
        self._vel: float = 0.0

    def step(self, kp: float, kd: float, p_cmd: float, t_ff: float) -> None:
        """Advance one control cycle."""
        tau = kp * (p_cmd - self.p_fb) + kd * (0.0 - self._vel) + t_ff
        tau = max(-32.0, min(32.0, tau))
        acc = (tau - self.b * self._vel) / self.J
        self._vel += acc * self.dt
        self.p_fb += self._vel * self.dt
        self.v_fb = self._vel
        self.t_fb = tau
```

### Running with plots

```
python motor_terminal.py --simulate --physics --plot
```

The `--plot` flag produces a matplotlib figure of position and velocity over time
after each `spin_once` command. No plot libraries are needed for Level A.

---

## Level C — CAN Frame Emulator

### Purpose

The real ESP32-S3 firmware runs completely unmodified. A Python process connected
to a USB-CAN adapter listens on the physical CAN bus and replies with synthetic
MIT feedback frames. This tests:

- CAN message packing in `pack_mit_command()` under edge-case inputs
- MIT vs servo frame detection logic in `can_receive_task()`
- Auto-ID adoption in the servo receive branch
- FreeRTOS race conditions between `can_rx` and `ctrl` tasks under real timing

### Hardware required

- USB-CAN adapter: PEAK PCAN-USB (~$50) or Canable v2 (~$15)
- CAN bus termination resistors (120Ω at each end)
- Connected to the same CAN bus as the ESP32-S3

### Architecture

```
ESP32-S3 (real firmware)  <--CAN bus-->  Python CAN emulator
                                            - receives MIT command frames
                                            - decodes position/velocity/gain commands
                                            - integrates Level B physics model
                                            - sends MIT feedback frame back
```

### MIT feedback frame format (what emulator sends)

```
Byte 0:   motor_id (CAN frame data[0])
Byte 1:   position high byte  (16-bit, range P_MIN..P_MAX)
Byte 2:   position low byte
Byte 3:   velocity high nibble (12-bit, range V_MIN..V_MAX)
Byte 4:   velocity low nibble | kp msb nibble
Byte 5:   kd high nibble (12-bit field)
Byte 6:   temperature (°C + 40 offset)
Byte 7:   error flags (0 = no error)
```

This is the exact format that `can_receive_task()` in `main.c` expects in the
MIT (non-extended) receive branch.

### Python library

Use the `python-can` library with the PCAN interface:

```python
import can

bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)

for msg in bus:
    if msg.is_extended_id:
        continue
    if msg.dlc == 8 and msg.data[6] == 0xFF and msg.data[7] == 0xFC:
        # enter motor mode command - start emitting feedback
        start_feedback(bus, motor_id=msg.arbitration_id)
```

### Test scenarios enabled

- Full round-trip test: `enable → spin_once → SPIN_DONE` with real firmware,
  no motor
- Feedback timeout fault: stop emitting frames mid-spin, verify `FAULT reason=fb_timeout`
  appears in firmware log within 200ms
- Frame corruption: send a feedback frame with DLC < 6, verify firmware ignores it
- Multi-ID conflict: send feedback from two different CAN IDs, verify only the
  correct one is accepted

---

## Integration with `motor_terminal.py`

All three levels are reached through the same CLI entry point:

```
python motor_terminal.py --port /dev/cu.usbmodem101       # real hardware
python motor_terminal.py --simulate                        # Level A
python motor_terminal.py --simulate --physics              # Level B
python motor_terminal.py --simulate --physics --plot       # Level B with plots
# Level C requires no flag change in motor_terminal.py - real firmware, emulated motor
```

The `--simulate` flag replaces the `serial.Serial` object with a `SimulatedMotor`
instance that exposes the same `write(bytes)` / `readline()` interface. The rest
of `motor_terminal.py` — the reader thread, command loop, `/raw` toggle — is
unchanged.

---

## Simulation Level Comparison

| Capability                          | Level A | Level B | Level C |
| ----------------------------------- | :-----: | :-----: | :-----: |
| Command state machine testing       |   yes   |   yes   |  yes\*  |
| Gain sensitivity / overshoot plots  |   no    |   yes   |   no    |
| Trajectory planning (ramp profiles) |   no    |   yes   |   no    |
| Feedback timeout fault path         | inject  | inject  |  real   |
| Firmware CAN parsing bugs           |   no    |   no    |   yes   |
| FreeRTOS task timing issues         |   no    |   no    | partial |
| Hardware required                   |  none   |  none   | adapter |

\*Level C tests the full firmware, so command behavior is tested indirectly.

---

## Phased delivery

| Phase     | Simulation work                                        |
| --------- | ------------------------------------------------------ |
| Phase 1   | Level A: scripted tests for fault state, stop, timeout |
| Phase 3.1 | Level B: gain sweep for configurable KP/KD commands    |
| Phase 3.2 | Level B: ramp trajectory validation                    |
| Phase 4   | Level C: multi-motor CAN emulator, e-stop verification |
