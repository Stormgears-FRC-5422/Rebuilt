# Intake Homing Fix — Analysis, Evidence & Implementation Plan

## Problem Statement

After adding springs to the mechanical intake system, the homing sequence produces a ~2.5° encoder offset. The arm physically reaches the home position (hard stop), but the encoder reads 2.5° instead of 0°. With `targetDegreesTolerance` tightened to 0.5°, `isAtTarget()` returns false, causing cascading command failures.

---

## Root Cause Analysis

Two issues compound to produce the 2.5° error:

### Issue 1: CAN Bus Timing Race in `declareHome()`

**The code sequence in `declareHome()` + `holdCurrentPosition(0.0)`:**
```java
// declareHome() — Intake.java:324
updownIntakeMotor.getConfigurator().setPosition(0.0);   // ← fire-and-forget, no timeout
// ...
// holdCurrentPosition(0.0) — called immediately after
setAngleDegrees(0.0);  // → updownIntakeMotor.setControl(mmRequest.withPosition(0))
```

**The problem:** `setPosition(0.0)` without a timeout parameter is **asynchronous** — it sends the CAN message and returns immediately without waiting for device acknowledgment. The subsequent `setControl(MotionMagic to 0°)` can arrive at the TalonFX **before** the position zero has been processed.

When this happens, two things go wrong:

1. **Wrong-direction kick:** The TalonFX computes the MotionMagic profile using the **pre-zero position** (~-17° raw), generating a trajectory from -17° to 0° — which is in the **positive direction** (away from home). The motor applies +0.77V in the first frame, actively pushing the arm away from home.

2. **Corrupted encoder reference:** When `setPosition(0.0)` finally arrives at the TalonFX, the arm has already moved away from the hard stop. The TalonFX sets 0° = the arm's **current position** (not the hard stop). This means the hard stop ends up at roughly -0.5° to -1° on the encoder, and **every subsequent position command is offset** from reality. This is the more serious consequence — the entire position reference frame is wrong.

**Supporting evidence:**

1. **CTRE API design:** `setPosition()` accepts an optional timeout parameter (`setPosition(0.0, timeoutSeconds)`). Without it, the call is non-blocking. With a timeout, it blocks until the device confirms. ([CTRE API Overview](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/api-overview.html))

2. **Community reports:** Multiple FRC teams on Chief Delphi reported `setPosition` not taking effect immediately — one solved it by adding a 1.0s timeout, another by spawning a thread with a 2.5s delay. ([Chief Delphi thread](https://www.chiefdelphi.com/t/setposition-not-working/463158))

3. **Best practice already in codebase:** `ModuleIOTalonFX.java:118` uses the blocking pattern with retry:
   ```java
   tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));
   ```
   The Intake's `declareHome()` did not follow this pattern.

### Issue 2: Reverse Soft Limit at 1° Blocks Recovery

After `declareHome()` enables soft limits, the reverse soft limit was set to `topIntakeLimit = 1°`. This means the motor controller will not apply reverse output (toward home) when the arm is at or below 1°.

After the arm rebounds to ~2.5° (from springs + the wrong-direction kick), the motor tries to drive back toward 0°. But as it approaches 1°, the soft limit cuts reverse output. The effective target becomes 1° instead of 0°, and the PID output (~0.35V) is insufficient to overcome the spring force pulling the arm away. The arm settles at ~2.5° — the equilibrium between the spring and the weakened PID response.

---

## Evidence from WPILog Data

**Log file:** `d.wpilog` — homing sequence at ~824s

### Before homing (stall at hard stop)
```
824.702 | StatorCurrent = 28.86A | Voltage = -0.51V    <- stalled at hard stop
824.723 |                        | Voltage = -0.49V
824.762 |                        | Voltage = -0.48V
```
Motor is stalled with high current, applying ~-0.5V (reduced from -2.0V by stator current limit at 25A).

### The critical transition
```
824.809 | Encoder = 0.000° | State -> HOLD | Voltage = 0.0V   <- declareHome() zeros encoder
824.829 | Encoder = 0.879° | Voltage = +0.77V                 <- WRONG DIRECTION!
824.849 | Encoder = 2.109° | Voltage = +0.02V                 <- overshooting
824.869 | Encoder = 2.637° | Voltage = -0.41V                 <- finally reverses
```

**At 824.829:** The motor applies **+0.77V** — driving the arm **away** from home. This is consistent with MotionMagic computing a profile from the pre-zero position (~-17°) to 0° in the positive direction, because `setPosition(0.0)` hasn't been processed by the TalonFX yet.

**At 824.869:** The motor reverses to -0.41V, but the arm has already reached 2.6°.

### Settling at ~2.5°
```
824.932 | Encoder = 2.549° | Voltage = -0.36V
824.949 | Encoder = 2.461° | StatorCurrent = 11.74A
824.969 | Encoder = 2.373° | Voltage = -0.35V
...
825.109 | Encoder = 2.549° | Voltage = -0.38V     <- stable at ~2.5°, only ~0.36V
```

The motor applies only ~-0.36V — insufficient to overcome the spring force. The arm settles at ~2.5° indefinitely.

### What happens next
```
825.809 | State -> PID_MOTION | TargetPosition = inf   <- operator commands intake down
826.029 | Encoder = 6.768°   | Voltage = 1.22V        <- arm moves away from home easily
```

When a real command arrives (BOTTOM position = infinity), the motor easily drives the arm with >1V. The motor has no problem generating force — the issue was the tiny PID error at 2.5° producing insufficient voltage.

---

## Krish's Previous Fixes (Mar 6)

For context, these commits addressed related but different aspects:

1. **`77a1847`** — Stall debouncing (5 frames of sustained current > 23A before declaring home). Prevents false homing from transient spikes.

2. **`fb5e293`** — Explicit `holdCurrentPosition(0.0)` target, tightened tolerance to 0.5°, added `timeBeforeHome = 0.2` property (not yet wired up).

3. **`1dbb694`** — Added `setAngleDegrees()` inside `holdCurrentPosition()` to immediately issue MotionMagic in the same frame. This was the right instinct — eliminate the one-frame gap — but the CAN timing race means the position zero still may not have arrived at the TalonFX when this command is processed.

---

## Fixes Applied

### Fix 1: Add timeout to `setPosition()` in `declareHome()`

**File:** `src/main/java/frc/robot/subsystems/Intake.java`

```java
// Before (fire-and-forget):
updownIntakeMotor.getConfigurator().setPosition(0.0);

// After (blocks until device confirms, up to 250ms):
updownIntakeMotor.getConfigurator().setPosition(0.0, 0.250);
```

This ensures the encoder zero is confirmed on the TalonFX before any subsequent MotionMagic command can be issued.

### Fix 2: Change reverse soft limit from 1° to 0°

**File:** `src/main/deploy/rebuilt.properties`

```properties
# Before:
intake.topIntakeLimit = (number) 1

# After:
intake.topIntakeLimit = (number) 0
```

This allows the motor to command all the way to 0° (home position).

### Fix 3: Move soft limits into the state machine

**File:** `Intake.java` — `setState()`

Moved `enableSoftLimits(true)` out of `declareHome()` and into the state machine. Soft limits are now managed symmetrically:
- **Entering HOMING** → `enableSoftLimits(false)` (already existed)
- **Entering HOLD / PID_MOTION / SIMPLE_MOTION** → `enableSoftLimits(true)` (added)

This ensures soft limits are re-enabled whenever the intake transitions to any operational state, not just after homing. The `declareHome()` method no longer touches soft limits.

---

## Out of Scope: PID Tuning for Spring Compensation

The springs create a position-dependent force that the current PID (kG=0, kS=0.20) doesn't fully compensate for. After these fixes, the motor may still settle a fraction of a degree away from 0° due to spring force vs. PID equilibrium. This is **not critical** — the important thing is that the encoder zero is correct so the system knows where the arm actually is. The spring force varies across the range of motion and is not easily modeled with the existing feedforward terms. If this needs addressing later, it would be a separate tuning effort.
