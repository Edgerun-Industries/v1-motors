# Motor Control and Packet Concepts

All examples and numbers in this document come directly from the v1-motors firmware
(`main/main.c`) and the OpenExo reference implementation (`ExoCode/src/Motor.cpp`).

---

## 1. CAN Bus

### What it is

CAN (Controller Area Network) is a serial bus protocol designed for embedded systems
where many nodes share two wires (CAN_H and CAN_L). Every node can transmit and all
nodes hear every message. A node ignores messages not addressed to it.

In v1-motors:

- TX: GPIO 4, RX: GPIO 5
- Baud rate: 1 Mbps
- Driver: ESP-IDF TWAI (Two-Wire Automotive Interface), the ESP32 name for CAN

### Frame types

| Type     | ID bits               | Max data | Used for                         |
| -------- | --------------------- | -------- | -------------------------------- |
| Standard | 11 bits (0x001–0x7FF) | 8 bytes  | MIT motor commands and feedback  |
| Extended | 29 bits               | 8 bytes  | Servo mode commands and feedback |

The firmware detects which type was received by checking `TWAI_MSG_FLAG_EXTD` in
`rx_msg.flags`.

### Arbitration

When two nodes transmit simultaneously, the one with the lower CAN ID wins and
continues uninterrupted. The losers back off and retry. This is deterministic and
lossless — a higher-priority message (lower ID) never corrupts a lower-priority one.

The CubeMars motor ID is configurable (1–127). Lower IDs have bus priority. For
multi-motor systems plan IDs intentionally — the motor closest to the joint with
the tightest control loop should have the lowest ID.

### Termination

Each end of the CAN bus requires a 120 Ω resistor across CAN_H and CAN_L. Without
it, reflected signals cause bit errors. A two-node bench setup (ESP32 + motor)
needs exactly two resistors — one at each end.

---

## 2. MIT Motor Protocol

### Background

MIT (Massachusetts Institute of Technology) developed this protocol for their
Mini-Cheetah legged robot actuators, later commercialized by CubeMars. It packs
position, velocity, two gains, and a torque feedforward into one 8-byte CAN frame
at up to 1000 frames/second.

### Command frame (controller → motor)

Sent as a standard CAN frame with `identifier = motor_id` (11-bit).

```
Byte:  0        1        2        3        4        5        6        7
       P[15:8]  P[7:0]   V[11:4]  V[3:0]   KP[11:4] KP[3:0]  KD[11:4] KD[3:0]
                                   KP[11:8]          KD[11:8]  T[11:8]  T[7:0]
```

More precisely, the 5 fields are packed across 64 bits as:

| Field                  | Bits | Position in output |
| ---------------------- | ---- | ------------------ |
| P (position target)    | 16   | bits 63–48         |
| V (velocity target)    | 12   | bits 47–36         |
| KP                     | 12   | bits 35–24         |
| KD                     | 12   | bits 23–12         |
| T (torque feedforward) | 12   | bits 11–0          |

Actual packing from `pack_mit_command()` in [main.c](main/main.c):

```c
out[0] = p_int >> 8;
out[1] = p_int & 0xFF;
out[2] = v_int >> 4;
out[3] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
out[4] = kp_int & 0xFF;
out[5] = kd_int >> 4;
out[6] = ((kd_int & 0x0F) << 4) | (t_int >> 8);
out[7] = t_int & 0xFF;
```

### Fixed-point encoding

All five values are transmitted as unsigned integers mapped from a physical range.
The firmware uses this conversion in both directions:

```
float → uint:  uint = (x - x_min) / (x_max - x_min) * (2^bits - 1)
uint → float:  x    = uint * (x_max - x_min) / (2^bits - 1) + x_min
```

Physical ranges (from the firmware `#define` block):

| Field        | Min         | Max         | Bits | Resolution           |
| ------------ | ----------- | ----------- | ---- | -------------------- |
| P (position) | −12.5 rad   | +12.5 rad   | 16   | 0.00038 rad ≈ 0.022° |
| V (velocity) | −37.5 rad/s | +37.5 rad/s | 12   | 0.018 rad/s          |
| KP           | 0           | 500 Nm/rad  | 12   | 0.122 Nm/rad         |
| KD           | 0           | 5 Nm·s/rad  | 12   | 0.0012 Nm·s/rad      |
| T (torque)   | −32 Nm      | +32 Nm      | 12   | 0.016 Nm             |

### Feedback frame (motor → controller)

Sent by the motor as a standard CAN frame with `identifier = motor_id`.

```
Byte 0:  motor_id  (echoed)
Byte 1:  P[15:8]   position high byte
Byte 2:  P[7:0]    position low byte
Byte 3:  V[11:4]   velocity high byte
Byte 4:  V[3:0] | KP[11:8]   (nibble-packed)
Byte 5:  KP[7:0]
Byte 6:  temperature (°C + 40 offset, so 0x00 = −40°C, 0x59 = 49°C)
Byte 7:  error flags
```

Decoding from the firmware receive handler:

```c
p_int = (data[1] << 8) | data[2];
v_int = (data[3] << 4) | (data[4] >> 4);
i_int = ((data[4] & 0x0F) << 8) | data[5];
temp  = data[6] - 40;
err   = data[7];
```

### Special command frames

Two 8-byte magic values put the motor in or out of motor mode. Both are sent as
standard CAN frames to the motor's ID:

| Purpose          | Bytes                     |
| ---------------- | ------------------------- |
| Enter motor mode | `FF FF FF FF FF FF FF FC` |
| Exit motor mode  | `FF FF FF FF FF FF FF FD` |

The motor does not respond until it receives Enter motor mode. Sending any MIT
command while in exit state is ignored. This is why `enable` must be sent before
`spin_once` and why `fb_seen` is cleared on re-enable.

---

## 3. Servo Protocol (Extended CAN)

Some CubeMars firmware versions respond on extended CAN frames instead of MIT mode.
The firmware auto-detects which protocol the motor is using based on the
`TWAI_MSG_FLAG_EXTD` bit on the first received frame.

### `spin_once` in servo mode

CAN ID format: `(motor_id & 0xFF) | (packet_type << 8)`

For a position command, `packet_type = 4` (CAN_PACKET_SET_POS):

```c
can_id = (motor_id & 0xFF) | (4 << 8);
```

Position encoded as a 32-bit integer in units of 0.000001 degrees (1 µdeg):

```c
pos_cmd = lroundf(target_deg * 1000000.0f);
payload[0..3] = pos_cmd big-endian
```

### Servo feedback frame

Received as an extended CAN frame. CAN ID low byte is the motor ID.

```
Byte 0–1:  position (int16, units: 0.1°)
Byte 2–3:  speed    (int16, units: 10 eRPM)
Byte 4–5:  current  (int16, units: 0.01 A)
Byte 6:    temperature (°C, raw)
Byte 7:    error flags
```

---

## 4. Impedance Control

### What it is

Impedance control makes a motor behave like a programmable spring-damper. Instead
of commanding a fixed torque, you specify:

- A target position (the equilibrium of the spring)
- A stiffness KP (spring constant, Nm/rad — how hard it resists displacement)
- A damping KD (damper coefficient, Nm·s/rad — how hard it resists velocity)
- A torque feedforward T_ff (bias torque regardless of position/velocity)

The motor computes torque internally each cycle:

$$\tau = K_p \cdot (\theta_{target} - \theta_{actual}) + K_d \cdot (0 - \dot{\theta}_{actual}) + \tau_{ff}$$

This is exactly what the MIT protocol encodes. The 8-byte frame is not just a
position command — it is a full impedance setpoint updated every control cycle.

### Why this matters for exoskeletons

Pure position control (KP very high, KD very high) makes the joint rigid —
dangerous if the wearer's limb is not exactly where the controller expects.
Pure torque control (KP=0, KD=0, T_ff=x) applies a constant force but cannot
resist drift.

Impedance control is the middle ground: you tune KP and KD so the joint feels
like a "soft" spring when the wearer pushes against it, absorbing impact rather
than fighting the user.

### Current firmware gains

`spin_once` uses KP=20 Nm/rad, KD=1 Nm·s/rad. These are reasonable starting
values for a bench test but have not been calibrated for any specific joint.
After SPIN_DONE, the motor holds position with the same gains.

### Gain tuning intuition

| Symptom                                                  | Likely cause                       | Fix                                  |
| -------------------------------------------------------- | ---------------------------------- | ------------------------------------ |
| Motor oscillates around target                           | KD too low                         | Increase KD                          |
| Motor moves sluggishly, overshoots and oscillates slowly | KP too high for given KD           | Reduce KP or increase KD             |
| Motor holds position fine but feels very stiff to push   | KP too high for wearable use       | Reduce KP                            |
| Motor drifts from target position when loaded            | KP too low                         | Increase KP                          |
| Loud buzzing / chattering noise                          | High gains + control loop too slow | Reduce KP or increase loop frequency |

---

## 5. Control Loop Frequency

The firmware runs at 200 Hz (`CONTROL_PERIOD_MS = 5`). Every 5 ms, the control
task reads the latest feedback, checks for faults, and sends one MIT command frame.

### Why frequency matters

The control law updates the impedance setpoint at fixed intervals. Between updates
the motor applies the last commanded gains. If a disturbance arrives during a 20 ms
interval (50 Hz, the old value), the motor's response is already 20 ms stale by
the time the next correction is computed.

For human-robot contact, 50 Hz is marginal for stable impedance control. 200 Hz
(5 ms) is a safe minimum; OpenExo runs at 500 Hz for similar reasons.

If you increase KP significantly in future phases, increase loop frequency first.
The relationship between KP, KD, and minimum stable loop frequency follows:

$$f_{min} \approx \frac{1}{2\pi} \sqrt{\frac{K_p}{J}}$$

where J is the rotor inertia. For KP=20 and J≈0.001 kg·m², this gives approximately
22 Hz — so 200 Hz provides a comfortable margin. At KP=200, the minimum rises to
~71 Hz, meaning 200 Hz is still safe but the margin has shrunk.

---

## 6. Fault and Safety State Machine

### States (after Phase 1 fixes)

```
IDLE (enabled=false, fault=false)
  │
  ├─ enable ──────────────────────────────────────> ENABLED
  │                                                      │
  │         feedback arrives each cycle                  │  feedback silent >200ms
  │         spin_once sets spin_active=true              │
  │                                                      ▼
  │                                                  FAULT
  │                                               (enabled=false)
  │                                                      │
  │         disable ◄─────────────────────────────────────
  │         (clears fault)
  │
  ├─ stop ──────────────────────────────────────> IDLE + CMD_EXIT_MOTOR_MODE sent
  │
  └─ disable ───────────────────────────────────> IDLE + CMD_EXIT_MOTOR_MODE sent
```

### Key transitions

| Event                 | From    | To      | CAN sent             |
| --------------------- | ------- | ------- | -------------------- |
| `enable`              | IDLE    | ENABLED | CMD_ENTER_MOTOR_MODE |
| `enable`              | FAULT   | FAULT   | nothing (NACK)       |
| `disable`             | any     | IDLE    | CMD_EXIT_MOTOR_MODE  |
| `stop`                | any     | IDLE    | CMD_EXIT_MOTOR_MODE  |
| feedback silent 200ms | ENABLED | FAULT   | CMD_EXIT_MOTOR_MODE  |
| SPIN_TIMEOUT (8s)     | ENABLED | IDLE    | CMD_EXIT_MOTOR_MODE  |

### Feedback timeout (200 ms)

Defined by `FB_TIMEOUT_US 200000LL`. The control task checks every cycle:

```c
if (enabled && !fault && fb_seen)
    if (now - last_fb_time_us > FB_TIMEOUT_US)
        → fault=true, enabled=false, send CMD_EXIT_MOTOR_MODE
```

The check only fires after `fb_seen=true`. This means if the motor never responds
at all (wrong ID, disconnected cable before first message), no fault is generated —
the motor simply never moves. The operator sees `seen=0` in `status`.

---

## 7. MIT Frame Worked Example

Command: hold position at 0 rad, velocity target 0, KP=20, KD=1, T=0.

Step 1 — compute integers:

```
p_int  = float_to_uint(0.0,  -12.5, 12.5,  16) = 32767  (0x7FFF)
v_int  = float_to_uint(0.0,  -37.5, 37.5,  12) = 2047   (0x7FF)
kp_int = float_to_uint(20.0,  0.0,  500.0, 12) = 163    (0x0A3)
kd_int = float_to_uint(1.0,   0.0,  5.0,   12) = 819    (0x333)
t_int  = float_to_uint(0.0,  -32.0, 32.0,  12) = 2047   (0x7FF)
```

Step 2 — pack into bytes:

```
out[0] = 0x7F           (p_int >> 8)
out[1] = 0xFF           (p_int & 0xFF)
out[2] = 0x7F           (v_int >> 4)  = 0x7FF >> 4 = 0x7F
out[3] = 0xF0 | 0x00   (v_int low nibble << 4) | (kp_int >> 8)
       = 0xF0 | 0x00 = 0xF0
out[4] = 0xA3           (kp_int & 0xFF)
out[5] = 0x33           (kd_int >> 4) = 0x333 >> 4 = 0x33
out[6] = 0x37           ((kd_int & 0x0F) << 4) | (t_int >> 8)
       = (0x3 << 4) | 0x7 = 0x37
out[7] = 0xFF           (t_int & 0xFF)
```

Final frame bytes: `7F FF 7F F0 A3 33 37 FF`

---

## 8. CAN Multi-Motor Addressing

Each motor on the bus has a unique integer ID (1–127). The firmware sends:

- Standard frame with `identifier = motor_id` for MIT commands
- Standard frame with `identifier = motor_id` for Enter/Exit motor mode

The motor sends feedback with `identifier = motor_id` (standard) or
`identifier = (packet_type << 8) | motor_id` (extended/servo).

To run multiple motors on one bus:

1. Assign unique IDs to each motor using the CubeMars configuration tool before
   connecting them to the same bus.
2. Filter incoming frames by `motor_id` in the receive task — the firmware already
   does this: `if (identifier != motor_id && data[0] != motor_id) continue;`
3. Send control frames to each motor ID in sequence within the same control cycle.
   At 200 Hz (5 ms loop) and 1 Mbps CAN, sending 8 × 8-byte frames takes
   ~0.64 µs each — 8 motors costs about 5 µs total, well within the 5 ms budget.

CAN IDs used by OpenExo for reference (useful for planning joint assignments):

| Joint | Left ID | Right ID |
| ----- | ------- | -------- |
| Hip   | 65      | 33       |
| Knee  | 66      | 34       |
| Ankle | 68      | 36       |
| Elbow | 72      | 40       |

---

## 9. FreeRTOS Task Architecture

Three tasks run concurrently, pinned to specific cores:

| Task     | Priority    | Core | Role                                                              |
| -------- | ----------- | ---- | ----------------------------------------------------------------- |
| `can_rx` | 1           | 1    | Blocks on `twai_receive()`, decodes CAN frames, updates `g_state` |
| `ctrl`   | 4 (highest) | 0    | Runs every 5 ms, checks fault, sends MIT command frame            |
| `serial` | 3           | 0    | Reads USB serial bytes, calls `handle_command()`                  |

`can_rx` runs on Core 1 so the CAN receive interrupt is not competing with the
control task for Core 0 cycles. The control task has the highest priority so it
preempts the serial task whenever its 5 ms timer fires.

All shared state (`g_state`, `g_motor_id`) is protected by `portMUX_TYPE g_lock`
using `portENTER_CRITICAL` / `portEXIT_CRITICAL`, which disables interrupts on both
cores for the duration of the critical section.

---

## 10. Glossary

| Term              | Meaning                                                                   |
| ----------------- | ------------------------------------------------------------------------- |
| CAN               | Controller Area Network — differential serial bus                         |
| TWAI              | Two-Wire Automotive Interface — ESP-IDF name for CAN peripheral           |
| MIT protocol      | 8-byte position/velocity/impedance command frame from MIT Cheetah project |
| DLC               | Data Length Code — number of data bytes in a CAN frame (0–8)              |
| eRPM              | Electrical RPM — motor shaft RPM × number of pole pairs                   |
| KP                | Proportional gain (spring stiffness), units Nm/rad                        |
| KD                | Derivative gain (damping), units Nm·s/rad                                 |
| T_ff              | Torque feedforward — constant torque added regardless of position error   |
| Impedance control | Control mode where the joint behaves as a programmable spring-damper      |
| Motor mode        | CubeMars operating state where MIT commands are accepted                  |
| fb_seen           | Flag set true on first valid feedback frame after enable                  |
| FAULT             | State entered when feedback is silent for >200 ms while enabled           |
