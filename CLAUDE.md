# Rebuilt Robot Codebase

## Critical Rule: Constants Are Auto-Generated

**NEVER edit `Constants.java` or `Options.java` directly.** They are auto-generated from property files by the Gradle build.

To add or change a constant:
1. Edit the appropriate `.properties` file in `src/main/deploy/` (e.g., `rebuilt.properties`)
2. Run `./gradlew build` -- the `generateCode` task regenerates `Constants.java` and `Options.java`
3. Property format: `section.name = (type) value` where type is `boolean`, `number`, `int`, `String`, or `option`

Example:
```properties
intake.myNewValue = (number) 42.0
intake.myToggle = (boolean, option) true   # "option" makes it dashboard-tunable
```

After build, access via `Constants.Intake.myNewValue` or `IntakeOptions.create().myToggle.get()`.

## Architecture Overview

This is a WPILib command-based FRC robot project (Team 5422 - Stormgears).

### Properties -> Constants Pipeline

```
rebuilt.properties  -->  ConstantGenerator (Gradle task)  -->  Constants.java + Options.java
```

- `StormProp` loads properties at runtime from `/home/lvuser/deploy/` on the RoboRIO
- Properties tagged with `(option)` become dashboard-tunable `Option` objects in `Options.java`
- Code generators live in `src/generate/java/frc/utils/generator/`

### Subsystems (`frc.robot.subsystems`)

All subsystems extend `StormSubsystem` (which extends WPILib's `SubsystemBase`). StormSubsystem adds:
- Throttled `console()` logging (prints every N iterations to reduce spam)
- Automatic SmartDashboard registration

Key subsystems:
- **Intake** -- arm position (MotionMagic) + roller motor, state machine (UNKNOWN/HOMING/HOLD/PID_MOTION/SIMPLE_MOTION/IDLE/DIAGNOSTIC)
- **ShooterHopper** -- flywheel + fuel injector, supports VelocityVoltage and TorqueCurrentFOC
- **Climber** -- leader-follower motors with PositionTorqueCurrentFOC
- **Drive** -- selected at runtime by `DrivetrainFactory` based on `Constants.Drive.driveType` (AkitDrive, BasicSwerve, etc.)
- **VisionSubsystem** -- Limelight AprilTag detection, consumed-on-read measurements
- **Lights** -- addressable LEDs with dual left/right views

Subsystems use explicit `enum` state machines. Commands drive state transitions.

### Commands (`frc.robot.commands`)

All commands extend `StormCommand` (which extends WPILib's `Command`). StormCommand adds:
- Lifecycle logging with FPGA timestamps
- `safeAddRequirements()` -- null-checks subsystems before adding requirements

Commands are organized by subsystem:
- `commands/intake/` -- IntakeHome, IntakeGoToPosition, RunRoller, IntakeManual, DynamicRumble, SimpleRumble
- `commands/climber/` -- ClimberHome, ClimberSequence
- `commands/diagnostic/` -- SystemCheck, ShooterDiagnostic, IntakeDiagnostic
- Top-level: JoyStickDrive, ShootCommand, AutoShootCommand, AlignNarrow, DriveCommands

Pattern: create separate `Auto*` commands for autonomous-only behavior rather than adding auto modes to teleop commands. Do not add public `reset()` methods on subsystems.

### RobotContainer

`RobotContainer.java` wires everything together:
1. Creates subsystems conditionally based on `Constants.Toggles.*` flags
2. Sets default commands (e.g., JoyStickDrive)
3. Binds joystick buttons to commands in `configureBindings()`
4. Builds auto routines via `AutoCommandFactory`

### Motor Wrappers

- **StormTalon** (extends TalonFX) -- cached status signals, AdvantageKit logging, fluent builder (`.withName()`, `.withLoggingScale()`, `.withClosedLoopLogging()`)
- **StormSparkMax** -- similar wrapper for REV motors

### Logging

Uses AdvantageKit (`org.littletonrobotics.junction.Logger`):
- `Logger.recordOutput("key", value)` for custom telemetry
- StormTalon handles motor signal logging automatically in `periodic()`

### Build

```bash
./gradlew build          # Full build (generates constants, compiles, deploys)
./gradlew deploy         # Deploy to RoboRIO
./gradlew simulateJava   # Run in simulation
```
