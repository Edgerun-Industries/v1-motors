"""
Level A state machine simulator for v1-motors firmware (Phase 1).

Mirrors the exact command/response protocol of main.c without hardware or physics.
In 'normal' mode p_fb converges to p_cmd in one tick (instant response).
Use inject commands to force fault or spin-timeout scenarios for testing.

Usage (interactive):
    python simulator.py

Usage (programmatic / pytest):
    from simulator import MotorSimulator
"""

import threading
import time
from typing import Callable

_P_MIN = -12.5
_P_MAX = 12.5
_KP_HOLD = 20.0
_KD_HOLD = 1.0

FB_TIMEOUT_US = 200_000
SPIN_TIMEOUT_US = 8_000_000


class MotorSimulator:
    """
    State machine mirror of main.c control_task and handle_command after Phase 1 fixes.

    All methods return a list of log strings in the exact format the firmware emits:
        "I (<ms>) cubemars: <message>"    (ESP_LOGI)
        "W (<ms>) cubemars: <message>"    (ESP_LOGW)

    Parameters
    ----------
    clock_fn:
        Callable returning current time in microseconds. Defaults to wall clock.
        Pass a mock in tests to control time without sleeping.
    """

    def __init__(self, clock_fn: Callable[[], int] | None = None) -> None:
        self._clock_fn: Callable[[], int] = clock_fn or (
            lambda: time.monotonic_ns() // 1000
        )

        # Mirror of motor_state_t
        self.enabled: bool = False
        self.fault: bool = False
        self.spin_active: bool = False
        self.target_p: float = 0.0
        self.spin_start_us: int = 0
        self.last_fb_time_us: int = 0

        self.p_cmd: float = 0.0
        self.v_cmd: float = 0.0
        self.kp_cmd: float = 0.0
        self.kd_cmd: float = 1.0
        self.t_cmd: float = 0.0

        self.fb_seen: bool = True
        self.p_fb: float = 0.0
        self.v_fb: float = 0.0
        self.t_fb: float = 0.0
        self.temp_fb: int = 25
        self.err_fb: int = 0

        self.tx_count: int = 0
        self.rx_count: int = 0
        self.motor_id: int = 1

        # 'normal' | 'frozen' | 'stale'
        # frozen: no feedback at all (triggers fault)
        # stale:  timing updates but position doesn't move (triggers spin timeout)
        self._fb_mode: str = "normal"

        self.last_fb_time_us = self._now_us()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _now_us(self) -> int:
        return self._clock_fn()

    def _now_ms(self) -> int:
        return self._now_us() // 1000

    def _log_i(self, msg: str) -> str:
        return f"I ({self._now_ms()}) cubemars: {msg}"

    def _log_w(self, msg: str) -> str:
        return f"W ({self._now_ms()}) cubemars: {msg}"

    # ------------------------------------------------------------------
    # Core loop (mirrors control_task in main.c)
    # ------------------------------------------------------------------

    def tick(self) -> list[str]:
        """
        Run one control cycle. Call every CONTROL_PERIOD_MS (5 ms) in interactive
        use, or manually in tests. Returns list of log lines emitted this cycle.
        """
        logs: list[str] = []
        now = self._now_us()

        # Simulated CAN receive task -----------------------------------------
        if self._fb_mode == "normal":
            self.fb_seen = True
            self.p_fb = self.p_cmd
            self.v_fb = 0.0
            self.last_fb_time_us = now
            self.rx_count += 1
        elif self._fb_mode == "stale":
            # Timing alive but position frozen (motor mechanically stuck)
            self.fb_seen = True
            self.last_fb_time_us = now
            self.rx_count += 1
        # 'frozen': nothing updates — simulates disconnected CAN cable

        # Fault detection (mirrors lines 294-303 in main.c) -------------------
        if self.enabled and not self.fault and self.fb_seen:
            if (now - self.last_fb_time_us) > FB_TIMEOUT_US:
                self.fault = True
                self.enabled = False
                self.spin_active = False
                logs.append(
                    self._log_w(f"FAULT reason=fb_timeout last_pos={self.p_fb:.3f}")
                )
                return logs

        # Control logic (mirrors lines 307-333 in main.c) ---------------------
        if self.enabled:
            if self.spin_active and self.fb_seen:
                pos_error = self.target_p - self.p_fb
                if abs(pos_error) < 0.08:
                    self.spin_active = False
                    self.p_cmd = self.target_p
                    self.kp_cmd = _KP_HOLD
                    self.kd_cmd = _KD_HOLD
                    self.t_cmd = 0.0
                    logs.append(self._log_i(f"SPIN_DONE pos={self.p_fb:.3f}"))
                elif (now - self.spin_start_us) > SPIN_TIMEOUT_US:
                    self.spin_active = False
                    self.enabled = False
                    logs.append(
                        self._log_w(
                            f"SPIN_TIMEOUT pos={self.p_fb:.3f} "
                            f"target={self.target_p:.3f}"
                        )
                    )
            if self.enabled:
                self.tx_count += 1

        return logs

    # ------------------------------------------------------------------
    # Command handler (mirrors handle_command in main.c)
    # ------------------------------------------------------------------

    def handle_command(self, line: str) -> list[str]:
        """
        Process one command. Returns list of log lines.
        Mirrors handle_command() in main.c plus simulator inject commands.
        """
        line = line.strip()
        logs: list[str] = []
        if not line:
            return logs

        if line == "ping":
            logs.append(self._log_i("PONG"))

        elif line == "status":
            logs.extend(self._status_lines())

        elif line == "help":
            logs.extend(self._help_lines())

        elif line in ("raw on", "raw off"):
            logs.append(self._log_i(f"ACK {line}"))

        elif line.startswith("id "):
            try:
                new_id = int(line.split()[1])
            except (IndexError, ValueError):
                logs.append(self._log_w("NACK id parse_error"))
                return logs
            if not (1 <= new_id <= 127):
                logs.append(self._log_w("NACK id must be 1..127"))
            else:
                self.motor_id = new_id
                self.fb_seen = False
                self.rx_count = 0
                self.spin_active = False
                logs.append(self._log_i(f"ACK id={new_id}"))

        elif line == "scan_ids":
            logs.append(self._log_i("SCAN start ids=1..127"))
            logs.append(self._log_i("SCAN done"))

        elif line == "enable":
            if self.fault:
                logs.append(
                    self._log_w("NACK enable reason=fault_active send_disable_first")
                )
            else:
                self.enabled = True
                self.fb_seen = False  # Mirrors main.c: wait for fresh feedback frame
                self.last_fb_time_us = self._now_us()
                logs.append(self._log_i("ACK enable"))

        elif line == "disable":
            self.enabled = False
            self.fault = False
            self.spin_active = False
            logs.append(self._log_i("ACK disable"))

        elif line == "stop":
            self.enabled = False
            self.spin_active = False
            self.v_cmd = 0.0
            self.kp_cmd = 0.0
            self.kd_cmd = 0.0
            self.t_cmd = 0.0
            logs.append(self._log_i("ACK stop"))

        elif line in ("spin_once cw", "spin_once ccw"):
            if not self.fb_seen:
                logs.append(self._log_w("NACK spin_once reason=no_feedback"))
            elif self.fault:
                logs.append(self._log_w("NACK spin_once reason=fault_active"))
            else:
                sign = 1.0 if line == "spin_once cw" else -1.0
                self.target_p = self.p_fb + sign * 6.2831853
                self.enabled = True
                self.spin_active = True
                self.spin_start_us = self._now_us()
                self.p_cmd = self.target_p
                self.kp_cmd = _KP_HOLD
                self.kd_cmd = _KD_HOLD
                self.t_cmd = 0.0
                logs.append(self._log_i(f"ACK spin_once target={self.target_p:.3f}"))

        # Simulator inject commands -------------------------------------------
        elif line == "inject_fb_freeze":
            self._fb_mode = "frozen"
            logs.append(
                self._log_i("SIM inject_fb_freeze: feedback frozen (fault in ~200ms)")
            )

        elif line == "inject_position_freeze":
            self._fb_mode = "stale"
            logs.append(
                self._log_i("SIM inject_position_freeze: position frozen, timing alive")
            )

        elif line == "inject_restore":
            self._fb_mode = "normal"
            self.last_fb_time_us = self._now_us()
            logs.append(self._log_i("SIM inject_restore: feedback restored"))

        else:
            logs.append(self._log_w(f"NACK unknown_cmd={line}"))

        return logs

    # ------------------------------------------------------------------
    # Status / help (mirrors print_status, print_help in main.c)
    # ------------------------------------------------------------------

    def _status_lines(self) -> list[str]:
        return [
            self._log_i(
                f"STATUS id={self.motor_id} enabled={int(self.enabled)} "
                f"fault={int(self.fault)} tx={self.tx_count} rx={self.rx_count} "
                f"seen={int(self.fb_seen)} pos={self.p_fb:.3f} vel={self.v_fb:.2f} "
                f"tq={self.t_fb:.2f} temp={self.temp_fb} err={self.err_fb}"
            )
        ]

    def _help_lines(self) -> list[str]:
        entries = [
            "Commands:",
            "  ping",
            "  status",
            "  id <1..127>",
            "  scan_ids",
            "  raw on|off",
            "  enable",
            "  disable",
            "  stop",
            "  spin_once cw",
            "  spin_once ccw",
            "Simulator inject commands:",
            "  inject_fb_freeze       - stop feedback (fault fires after 200ms)",
            "  inject_position_freeze - freeze position only (spin timeout after 8s)",
            "  inject_restore         - restore normal feedback",
            "Note: FAULT (fb_timeout >200ms) disables motor. Send 'disable' to clear.",
        ]
        return [self._log_i(e) for e in entries]


# ----------------------------------------------------------------------
# Interactive runner
# ----------------------------------------------------------------------


def _run_interactive(sim: MotorSimulator, tick_interval_s: float = 0.005) -> None:
    stop_event = threading.Event()

    def _ticker() -> None:
        while not stop_event.is_set():
            for line in sim.tick():
                print(line)
            time.sleep(tick_interval_s)

    t = threading.Thread(target=_ticker, daemon=True)
    t.start()

    for line in sim.handle_command("help"):
        print(line)
    print(f"I ({sim._now_ms()}) cubemars: READY (simulator mode)")

    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break
            if cmd in ("quit", "/quit"):
                break
            for line in sim.handle_command(cmd):
                print(line)
    finally:
        stop_event.set()


if __name__ == "__main__":
    _run_interactive(MotorSimulator())
