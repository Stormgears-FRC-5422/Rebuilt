package frc.robot.subsystems.drive.basicSwerve.config;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public final class SwerveModuleConstants {
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_RIGHT = 2;
    public static final int BACK_LEFT = 3;
    public static final int NUM_MODULES = 4;

    public static final ModuleConfig flModuleConfig;
    public static final ModuleConfig frModuleConfig;
    public static final ModuleConfig blModuleConfig;
    public static final ModuleConfig brModuleConfig;

    public static final PidConfig drivePidConfig;
    public static final PidConfig steerPidConfig;

    public static ModuleConfig[] getOrderedModuleConfigs() {
        ModuleConfig[] configs = new ModuleConfig[NUM_MODULES];
        configs[FRONT_LEFT] = flModuleConfig;
        configs[FRONT_RIGHT] = frModuleConfig;
        configs[BACK_RIGHT] = brModuleConfig;
        configs[BACK_LEFT] = blModuleConfig;
        return configs;
    }

    private static ModuleConfig getDefaultModuleConfig() {
        return new ModuleConfig()
                .withDriveCoast(Drive.driveCoastMode)
                .withSteerCoast(Drive.steerCoastMode)
                .withSteerInverted(Drive.invertSteer)
                .withDriveRatio(Drive.driveRatio)
                .withSteerRatio(Drive.steerRatio)
                .withInvertEncoder(Drive.invertEncoder);
    }

    static {
        // Initialize PID configs
        drivePidConfig = new PidConfig()
                .withKP(Drive.driveKP)
                .withKI(Drive.driveKI)
                .withKD(Drive.driveKD)
                .withKMinMax(Drive.driveKMin, Drive.driveKMax);

        steerPidConfig = new PidConfig()
                .withKP(Drive.steerKP)
                .withKI(Drive.steerKI)
                .withKD(Drive.steerKD)
                .withKMinMax(Drive.steerKMin, Drive.steerKMax);

        // Save some space with these convenience variables
        double dx = Drive.drivetrainTrackwidthMeters / 2.0;
        double dy = Drive.drivetrainWheelbaseMeters / 2.0;

        // Initialize module configs
        flModuleConfig = getDefaultModuleConfig()
                .withName("Front Left")
                .withDriveID(Drive.frontLeftDriveID)
                .withSteerID(Drive.frontLeftSteerID)
                .withEncoderID(Drive.frontLeftEncoderID)
                .withWheelDiameter(Inches.of(Drive.frontLeftDiameter))
                .withDriveInverted(!Drive.invertRight)
                .withEncoderOffset(Degrees.of(Drive.frontLeftEncoderOffset))
                .withOffset(new Translation2d(dx, dy));

        frModuleConfig = getDefaultModuleConfig()
                .withName("Front Right")
                .withDriveID(Drive.frontRightDriveID)
                .withSteerID(Drive.frontRightSteerID)
                .withEncoderID(Drive.frontRightEncoderID)
                .withWheelDiameter(Inches.of(Drive.frontRightDiameter))
                .withDriveInverted(Drive.invertRight)
                .withEncoderOffset(Degrees.of(Drive.frontRightEncoderOffset))
                .withOffset(new Translation2d(dx, -dy));

        blModuleConfig = getDefaultModuleConfig()
                .withName("Back Left")
                .withDriveID(Drive.backLeftDriveID)
                .withSteerID(Drive.backLeftSteerID)
                .withEncoderID(Drive.backLeftEncoderID)
                .withWheelDiameter(Inches.of(Drive.backLeftDiameter))
                .withDriveInverted(!Drive.invertRight)
                .withEncoderOffset(Degrees.of(Drive.backLeftEncoderOffset))
                .withOffset(new Translation2d(-dx, dy));

        brModuleConfig = getDefaultModuleConfig()
                .withName("Back Right")
                .withDriveID(Drive.backRightDriveID)
                .withSteerID(Drive.backRightSteerID)
                .withEncoderID(Drive.backRightEncoderID)
                .withWheelDiameter(Inches.of(Drive.backRightDiameter))
                .withDriveInverted(Drive.invertRight)
                .withEncoderOffset(Degrees.of(Drive.backRightEncoderOffset))
                .withOffset(new Translation2d(-dx, -dy));
    }

    private SwerveModuleConstants() {
        // Prevent instantiation
    }
}
