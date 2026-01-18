package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class DriveCommands {

    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    public DriveCommands(){}

//    /** Measures the robot's wheel radius by spinning in a circle (per-wheel). */
//    public static Command wheelRadiusCharacterization(DrivetrainBase drive) {
//        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
//        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
//
//        return Commands.parallel(
//            // Drive control sequence
//            Commands.sequence(
//                Commands.runOnce(() -> limiter.reset(0.0)),
//                Commands.run(
//                    () -> {
//                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
//                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
//                    },
//                    drive)),
//
//            // Measurement sequence
//            Commands.sequence(
//                Commands.waitSeconds(1.0),
//
//                Commands.runOnce(
//                    () -> {
//                        state.positions = drive.getWheelRadiusCharacterizationPositions();
//                        state.lastAngle = drive.getRotation();
//                        state.gyroDelta = 0.0;
//                    }),
//
//                Commands.run(
//                        () -> {
//                            Rotation2d rotation = drive.getRotation();
//                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
//                            state.lastAngle = rotation;
//                        })
//
//                    .finallyDo(
//                        () -> {
//                            double[] finalPositions =
//                                drive.getWheelRadiusCharacterizationPositions();
//
//                            double[] wheelDeltas = new double[4];
//                            double[] wheelRadii = new double[4];
//
//                            for (int i = 0; i < 4; i++) {
//                                wheelDeltas[i] =
//                                    Math.abs(finalPositions[i] - state.positions[i]);
//
//                                wheelRadii[i] =
//                                    (state.gyroDelta * Constants.Drive.driveBaseRadius)
//                                        / wheelDeltas[i];
//                            }
//
//                            NumberFormat formatter = new DecimalFormat("#0.000");
//                            System.out.println(
//                                "********** Per-Wheel Radius Characterization Results **********");
//                            System.out.println(
//                                "\tGyro Delta: "
//                                    + formatter.format(state.gyroDelta)
//                                    + " radians");
//
//                            for (int i = 0; i < 4; i++) {
//                                System.out.println(
//                                    "\tWheel "
//                                        + i
//                                        + " Radius: "
//                                        + formatter.format(wheelRadii[i])
//                                        + " meters ("
//                                        + formatter.format(
//                                        Units.metersToInches(wheelRadii[i]))
//                                        + " inches)");
//                            }
//                        })));
//    }
//
//    private static class WheelRadiusCharacterizationState {
//        double[] positions = new double[4];
//        Rotation2d lastAngle = Rotation2d.kZero;
//        double gyroDelta = 0.0;
//    }

    /** Measures the robot's wheel radius per wheel, updating every iteration. */
    public static Command wheelRadiusCharacterization(DrivetrainBase drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                Commands.waitSeconds(1.0), // wait for modules to orient

                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                }),

                // Update wheel radii every iteration
                Commands.run(() -> {
                        Rotation2d rotation = drive.getRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;

                        double[] currentPositions = drive.getWheelRadiusCharacterizationPositions();
                        double[] wheelDeltas = new double[4];
                        double[] wheelRadii = new double[4];

                        for (int i = 0; i < 4; i++) {
                            wheelDeltas[i] = Math.abs(currentPositions[i] - state.positions[i]);
                            wheelRadii[i] = (state.gyroDelta * Constants.Drive.driveBaseRadius) / wheelDeltas[i];

                            // Optional: log every iteration
                            Logger.recordOutput("Drive/Wheel" + i + "Delta", wheelDeltas[i]);
                            Logger.recordOutput("Drive/Wheel" + i + "Radius", wheelRadii[i]);
                        }
                    })

                    // Final print on command end
                    .finallyDo(() -> {
                        double[] finalPositions = drive.getWheelRadiusCharacterizationPositions();
                        double[] finalDeltas = new double[4];
                        double[] finalRadii = new double[4];

                        for (int i = 0; i < 4; i++) {
                            finalDeltas[i] = Math.abs(finalPositions[i] - state.positions[i]);
                            finalRadii[i] = (state.gyroDelta * Constants.Drive.driveBaseRadius) / finalDeltas[i];
                        }

                        NumberFormat formatter = new DecimalFormat("#0.000000");
                        System.out.println("********** Per-Wheel Radius Characterization Results **********");
                        System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");

                        for (int i = 0; i < 4; i++) {
                            System.out.println(
                                "\tWheel " + i + " Delta: " + formatter.format(finalDeltas[i]) +
                                    " radians, Radius: " + formatter.format(finalRadii[i]) +
                                    " meters (" + formatter.format(Units.metersToInches(finalRadii[i])) + " inches)"
                            );
                        }
                    })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }


    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterizationAll(DrivetrainBase drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                        limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drive.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = drive.getRotation();
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                            var rotation = drive.getRotation();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius = (state.gyroDelta * Constants.Drive.driveBaseRadius) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000");
                            System.out.println(
                                "********** Wheel Radius Characterization Results **********");
                            System.out.println(
                                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println(
                                "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                            System.out.println(
                                "\tWheel Radius: "
                                    + formatter.format(wheelRadius)
                                    + " meters, "
                                    + formatter.format(Units.metersToInches(wheelRadius))
                                    + " inches");
                        })));
    }

}
