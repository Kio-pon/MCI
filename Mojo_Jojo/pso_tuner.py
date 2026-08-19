#!/usr/bin/env python3
"""
pso_tuner.py

Runs PSO on your STM32 self-balancing robot over UART.

INSTALL:
    pip install pyswarms pyserial numpy

USAGE:
    python pso_tuner.py --port /dev/ttyUSB0

HOW IT WORKS:
    1. PSO picks a set of [Kp, Ki, Kd, alpha].
    2. Script sends "GAINS Kp Ki Kd alpha\\n" to STM32.
    3. STM32 waits for tilt gate (vertical), then runs 30-second trial.
    4. STM32 streams "A <angle>\\n" back every 100ms.
    5. Script scores the trial: penalises large angles, falls, noise.
    6. PSO uses score to pick better gains next round.
    7. After all iterations, best gains printed and saved.

SEARCH BOUNDS (tuned for position control):
    Kp    : 50  to 200
    Ki    : 0.0 to 0.1
    Kd    : 0.05 to 0.5
    alpha : 0.95 to 0.999
"""

import argparse
import time
import numpy as np
import serial
import pyswarms as ps
from pyswarms.single import GlobalBestPSO

# ------------------------------------------------------------------ #
# Config                                                               #
# ------------------------------------------------------------------ #
BAUD_RATE        = 115200
TRIAL_TIMEOUT_S  = 35       # slightly more than 30s trial on STM32
FALL_PENALTY     = 500.0    # added to score if robot falls
N_PARTICLES      = 10       # PSO swarm size
N_ITERATIONS     = 8        # PSO iterations (10*8 = 80 robot trials)
RESET_BANNER     = "BOOT"

# Search bounds: [Kp, Ki, Kd, alpha]
BOUNDS = (
    np.array([50.0,   0.0,   0.05,  0.950]),   # lower
    np.array([200.0,  0.1,   0.5,   0.999])    # upper
)

# PSO options
PSO_OPTIONS = {
    'c1': 0.5,   # cognitive (personal best weight)
    'c2': 0.3,   # social (global best weight)
    'w':  0.9    # inertia
}

# ------------------------------------------------------------------ #
# Serial helpers                                                       #
# ------------------------------------------------------------------ #
def open_port(port: str) -> serial.Serial:
    ser = serial.Serial(port, BAUD_RATE, timeout=1.0)
    time.sleep(2.0)   # wait for STM32 reset after USB connect
    deadline = time.time() + 5.0
    while time.time() < deadline:
        line = ser.readline().decode(errors='replace').strip()
        if line == RESET_BANNER:
            ser.reset_input_buffer()
            print(f"[INFO] Opened {port} at {BAUD_RATE} baud")
            return ser
    ser.reset_input_buffer()
    print(f"[INFO] Opened {port} at {BAUD_RATE} baud")
    return ser


def send_gains(ser: serial.Serial, kp: float, ki: float,
               kd: float, alpha: float) -> None:
    msg = f"GAINS {kp:.4f} {ki:.4f} {kd:.4f} {alpha:.4f}\n"
    ser.write(msg.encode())
    print(f"  → Sent gains")


def reopen_after_reset(ser: serial.Serial) -> serial.Serial:
    port = ser.port
    try:
        ser.close()
    except Exception:
        pass
    print("  ↺ STM32 reset detected, reopening serial link...")
    return open_port(port)


def wait_for_gain_ack(ser: serial.Serial, timeout_s: float = 5.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors='replace').strip()
        if not line:
            continue
        if line == RESET_BANNER:
            return False
        if line == "GAINS_OK":
            print("  ✓ Gains accepted by STM32")
            return True
        if line == "ERR":
            print("  ✗ STM32 rejected the gain packet")
            return False
        if line in ("Waiting for robot vertical...", "Ready. Starting trial."):
            continue
    print("  ✗ Timed out waiting for STM32 to confirm gains")
    return False


def run_trial(ser: serial.Serial,
              kp: float, ki: float,
              kd: float, alpha: float) -> float:
    """
    Sends gains, collects angle stream, returns a score.
    Lower score = better (PSO minimises).
    """
    while True:
        ser.reset_input_buffer()
        send_gains(ser, kp, ki, kd, alpha)

        if not wait_for_gain_ack(ser):
            ser = reopen_after_reset(ser)
            continue

        angles   = []
        fell     = False
        deadline = time.time() + TRIAL_TIMEOUT_S

        while time.time() < deadline:
            line = ser.readline().decode(errors='replace').strip()
            if not line:
                continue
            if line == RESET_BANNER:
                ser = reopen_after_reset(ser)
                break
            if line.startswith("A "):
                try:
                    angles.append(float(line[2:]))
                except ValueError:
                    pass
            elif line == "DONE":
                print(f"    ✓ Trial complete. Samples: {len(angles)}")
                return score(angles, fell)
            elif line == "FALL":
                fell = True
                print("    ✗ Robot fell.")
                return score(angles, fell)
            elif line in ("Waiting for robot vertical...", "Ready. Starting trial.", "ERR", "GAINS_OK"):
                # Status messages, ignore
                pass

        else:
            return score(angles, fell)


def score(angles: list, fell: bool) -> float:
    """
    Scoring function (minimise this).

    Components:
      - Mean squared angle   : penalises steady lean
      - Std deviation        : penalises oscillation
      - Fall penalty         : large flat penalty for falling
      - Short trial penalty  : if fewer than 10 samples, likely fell immediately
    """
    if fell or len(angles) < 10:
        return FALL_PENALTY + 1000.0

    arr = np.array(angles)
    mse = float(np.mean(arr ** 2))      # mean squared angle
    std = float(np.std(arr))            # oscillation measure
    short_penalty = max(0, (300 - len(arr)) * 2.0)  # reward longer balance

    total = mse * 10.0 + std * 5.0 + short_penalty
    if fell:
        total += FALL_PENALTY

    print(f"    Score: {total:.2f}  (mse={mse:.3f}, std={std:.3f}, samples={len(arr)})")
    return total


# ------------------------------------------------------------------ #
# PSO objective function                                               #
# ------------------------------------------------------------------ #
def make_objective(ser: serial.Serial):
    def objective(params: np.ndarray) -> np.ndarray:
        """Evaluate each particle in the swarm."""
        costs = []
        for i, p in enumerate(params):
            kp, ki, kd, alpha = p
            print(f"\n  Particle {i+1}/{len(params)}: Kp={kp:.2f} Ki={ki:.4f} Kd={kd:.3f} α={alpha:.4f}")
            c = run_trial(ser, kp, ki, kd, alpha)
            costs.append(c)
            time.sleep(1.0)   # short pause between trials
        return np.array(costs)
    return objective


# ------------------------------------------------------------------ #
# Main                                                                 #
# ------------------------------------------------------------------ #
def main():
    parser = argparse.ArgumentParser(description="PSO PID tuner for STM32 robot")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    args = parser.parse_args()

    ser = open_port(args.port)

    optimizer = GlobalBestPSO(
        n_particles=N_PARTICLES,
        dimensions=4,
        options=PSO_OPTIONS,
        bounds=BOUNDS
    )

    print(f"\n[PSO] Starting: {N_PARTICLES} particles × {N_ITERATIONS} iterations = {N_PARTICLES * N_ITERATIONS} trials")
    print(f"[PSO] Each trial: 30 seconds on robot + tilt gate\n")

    best_cost, best_pos = optimizer.optimize(
        make_objective(ser),
        iters=N_ITERATIONS,
        verbose=False
    )

    kp, ki, kd, alpha = best_pos
    print("\n" + "="*50)
    print("PSO TUNING COMPLETE")
    print("="*50)
    print(f"  Kp    = {kp:.6f}")
    print(f"  Ki    = {ki:.6f}")
    print(f"  Kd    = {kd:.6f}")
    print(f"  alpha = {alpha:.6f}")
    print(f"  Score = {best_cost:.4f}")
    print("="*50 + "\n")

    # Save to file
    with open("best_gains.txt", "w") as f:
        f.write(f"Kp={kp:.6f}\n")
        f.write(f"Ki={ki:.6f}\n")
        f.write(f"Kd={kd:.6f}\n")
        f.write(f"alpha={alpha:.6f}\n")
        f.write(f"score={best_cost:.4f}\n")
    print("[SAVE] Wrote best_gains.txt\n")

    ser.close()


if __name__ == "__main__":
    main()
