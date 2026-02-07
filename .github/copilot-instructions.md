# Copilot Instructions for Rebuilt (FRC Team 5422)

## Project Overview
FRC robot code for Team 5422 (Stormgears), built on WPILib 2026 with AdvantageKit logging. This is a Java 17 Gradle project targeting the RoboRIO.

## Architecture

### Configuration-Driven Constants System
**Critical**: `Constants.java` is auto-generated from `.properties` files — never edit it directly.

- `src/main/deploy/config.properties` — base config + robot-to-file mappings
- `src/main/deploy/{robotname}.properties` — robot-specific overrides (e.g., `rebuilt.properties`)
- `src/generate/java/frc/utils/generator/ConstantGenerator.java` — generates `Constants.java`
- Property format: `section.key = (type) value` (e.g., `drive.driveKP = (number) 0.5`)
- Run `./gradlew generateCode` or `./gradlew build` to regenerate

**Adding new constants**: Add to appropriate `.properties` file, rebuild to regenerate `Constants.java`.

### Runtime Options (Tunable Parameters)
`Options.java` is also auto-generated. Options marked with `(type, option)` in properties become runtime-tunable via `OptionsController`:
```java
// In properties: drive.useFieldRelative = (boolean, option) true
// Access: Options.DriveOptions.create().useFieldRelative.get()
```

### Subsystem Pattern
- Extend `StormSubsystem` (not `SubsystemBase`) for throttled console logging
- Extend `StormCommand` for commands with built-in logging
- Use `console("message")` for periodic-safe logging (auto-throttled)

### Drivetrain Architecture
Factory pattern with multiple implementations:
- `DrivetrainFactory` → creates `DrivetrainBase` subclass based on `Constants.Drive.driveType`
- Types: `BasicSwerve`, `DiagnosticSwerve`, `CTRSwerve`
- Subtypes for CTR: `CtrRebuilt` (maps to `RebuiltTunerConstants`)

### Joystick Factory
Similarly, `RebuiltJoystickFactory` creates joysticks based on config:
- Types: `XboxController`, `LogitechController`

## Key Files & Directories

| Path | Purpose |
|------|---------|
| `src/main/deploy/*.properties` | Robot configuration files |
| `src/main/java/frc/robot/` | Main robot code |
| `src/main/java/frc/utils/` | Reusable utilities |
| `src/generate/java/` | Code generators |
| `vendordeps/` | WPILib vendor dependencies |

## Build & Deploy Commands

```bash
./gradlew build          # Build + generate Constants.java
./gradlew deploy         # Deploy to RoboRIO
./gradlew simulateJava   # Run simulation with GUI
./gradlew test           # Run JUnit tests
```

## Simulation Modes
Controlled by `Constants.Toggles.useAdvantageKit` and `Constants.Akit.doReplay`:
- `REAL` — Physical robot
- `AKIT_SIM` — AdvantageKit simulation with NT4 logging
- `AKIT_REPLAY` — Replay from log files
- `SIMULATION` — Basic WPILib simulation

Simulation uses `simOverride` property for config overrides (e.g., `rebuilt.sim.properties`).

## Conventions

### Naming
- Subsystems in `frc.robot.subsystems.*`
- Commands in `frc.robot.commands.*`
- CAN IDs and hardware config in robot-specific `.properties` files

### Logging
- Use AdvantageKit `Logger.recordOutput()` for telemetry
- Use `@AutoLogOutput` annotation for automatic field logging
- Use `console()` methods in `StormSubsystem`/`StormCommand` for throttled output

### Vision (Scaffolded — Not Yet Active)
- `VisionSubsystem` exists using Limelight via `LimelightHelpers` (in `frc.utils.vision`)
- `FieldConstants.getPoseTag(id)` provides AprilTag poses for the 2025 Reefscape field (supports AndyMark and standard field variants)
- **Not yet wired in**: `VisionSubsystem` is not instantiated in `RobotContainer` and has no `toggles.useVision` property
- To activate: add `toggles.useVision` to `.properties` files, instantiate in `RobotContainer` guarded by the toggle, and connect to `DrivetrainBase`'s pose estimator

### Alliance & Field Handling
- `RobotState` singleton manages alliance, period, and simulation mode
- `RebuiltField.remapPose()` handles blue-to-red coordinate transformation
- Driver perspective handled automatically based on alliance when set to `"auto"`

## Dependencies
- **AdvantageKit** — Logging and replay
- **Phoenix6** — CTRE motor controllers (TalonFX, CANCoder, Pigeon2)
- **REVLib** — REV motor controllers (SparkMax)
- **Lombok** — Reduce boilerplate (enabled via Gradle plugin)
