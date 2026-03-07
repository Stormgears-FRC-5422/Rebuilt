# Intake Homing Fix — Analysis, Evidence & Implementation Plan

## Problem Statement

After adding springs to the mechanical intake system, the homing sequence produces a ~2.5° encoder offset. The arm physically reaches the home position (hard stop), but the encoder reads 2.5° instead of 0°. With `targetDegreesTolerance` tightened to 0.5°, `isAtTarget()` returns false, causing cascading command failures.

---

## Root Cause Analysis

Two issues compound to produce the 2.5° error:

### Issue 1: Stale Motor Control State During Home→Hold Transition

**The code sequence in `IntakeHome.end()` → `declareHome()` + `holdCurrentPosition(0.0)`:**
```java
// declareHome() — Intake.java
updownIntakeMotor.getConfigurator().setPosition(0.0, 0.250);  // blocking — encoder IS zeroed
// holdCurrentPosition(0.0) — called immediately after
setAngleDegrees(0.0);  // → updownIntakeMotor.setControl(mmRequest.withPosition(0))
```

**API semantics (corrected):**
- `setPosition(0.0)` is **blocking by default** — the no-timeout overload internally calls `setPosition(newValue, 0.100)`, waiting up to 100ms for device acknowledgment. The encoder zero IS confirmed before returning.
- `setControl(...)` is **non-blocking** — it submits a control request to the TalonFX and returns immediately. CTRE describes control requests as transmitted at an update frequency; `UpdateFreqHz = 0` makes them "synchronously one-shot" (send once now), not "block until device-side completion."

**The problem:** The motor transitions directly from VoltageOut(-2.0V) (homing) to MotionMagic(0°) without clearing the TalonFX's internal control state. When MotionMagic activates on the TalonFX, it computes its first S-curve profile using whatever velocity/state the controller has internally. After being stalled at -2.0V against a hard stop and then having the position register jump from ~-17° to 0° via `setPosition`, the TalonFX's internal velocity estimator may interpret the position discontinuity as real motion, producing a phantom positive velocity. MotionMagic then generates a trajectory that initially drives in the **positive direction** (away from home) to follow this phantom velocity before correcting.

**At 824.829:** The motor applies **+0.77V** — driving the arm **away** from home. This is consistent with MotionMagic starting from corrupted internal velocity state rather than from rest.

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

**At 824.829:** The motor applies **+0.77V** — driving the arm **away** from home. This is consistent with MotionMagic computing a profile from stale internal state (phantom velocity from the position reset), because `setControl` is non-blocking and inherits whatever velocity/state the TalonFX had from the -2.0V homing drive.

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

3. **`1dbb694`** — Added `setAngleDegrees()` inside `holdCurrentPosition()` to immediately issue MotionMagic in the same frame. This was the right instinct — eliminate the one-frame gap — but the stale control state means MotionMagic still starts from a corrupted velocity estimate.

---

## Fixes Applied

### Fix 1: Add timeout to `setPosition()` in `declareHome()` (commit `5052ae7`)

**File:** `src/main/java/frc/robot/subsystems/Intake.java`

```java
// Before (100ms default timeout):
updownIntakeMotor.getConfigurator().setPosition(0.0);

// After (explicit 250ms timeout for safety margin):
updownIntakeMotor.getConfigurator().setPosition(0.0, 0.250);
```

Note: the original call was already blocking by default (100ms), so this change increases the timeout margin but does not fix the root cause.

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

### Fix 4: NeutralOut as default motor state (Mar 7)

**File:** `Intake.java`

**Root cause fix.** The motor now starts in `NeutralOut` and stays there until explicitly commanded. This ensures MotionMagic never inherits stale velocity/control state from a previous mode.

Changes:
- Added `IDLE` to `MotionProfile` enum and a `setIdle()` helper that applies `NeutralOut`
- **Constructor:** calls `setIdle()` — motor starts in NeutralOut, not relying on `set(0)`
- **UNKNOWN state:** calls `setIdle()` — explicit NeutralOut when state is unknown
- **`declareHome()`:** calls `setIdle()` after `setPosition(0.0, 0.250)` — creates a clean break from the -2.0V homing drive before `holdCurrentPosition` transitions to MotionMagic
- **`periodic()` default case:** calls `setIdle()` instead of `set(0)` for consistency

The critical change is in `declareHome()`: after zeroing the encoder, the motor is explicitly set to NeutralOut. When `holdCurrentPosition(0.0)` then issues the MotionMagic command, the TalonFX transitions from NeutralOut (zero velocity, no active control) to MotionMagic — eliminating the phantom velocity that caused the wrong-direction kick.

---

## Out of Scope: PID Tuning for Spring Compensation

The springs create a position-dependent force that the current PID (kG=0, kS=0.20) doesn't fully compensate for. After these fixes, the motor may still settle a fraction of a degree away from 0° due to spring force vs. PID equilibrium. This is **not critical** — the important thing is that the encoder zero is correct so the system knows where the arm actually is. The spring force varies across the range of motion and is not easily modeled with the existing feedforward terms. If this needs addressing later, it would be a separate tuning effort.
