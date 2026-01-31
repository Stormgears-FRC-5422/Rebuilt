package frc.robot.subsystems.drive.basicSwerve;

import frc.robot.Constants;
import frc.robot.subsystems.drive.basicSwerve.config.ModuleConfig;
import frc.utils.pid.PidConfig;

/**
 * Factory for creating SwerveModule instances based on configuration.
 * Uses Constants.Drive.moduleType to determine which implementation to create.
 */
public class SwerveModuleFactory {

    /**
     * Create a swerve module instance based on configuration
     * @param index Module index (0-3)
     * @param config Module hardware configuration
     * @param drivePidConfig Drive motor PID configuration
     * @param steerPidConfig Steer motor PID configuration
     * @return Vendor-specific SwerveModuleBase implementation
     */
    public static SwerveModuleBase createModule(int index, ModuleConfig config,
                                                PidConfig drivePidConfig,
                                                PidConfig steerPidConfig) {
        String moduleType = Constants.Drive.moduleType;
        System.out.println("Creating module " + index + " (" + config.name + ") of type: " + moduleType);

        return switch (moduleType.toLowerCase()) {
            case "sparkmax" -> new SparkMaxSwerveModule(index, config, drivePidConfig, steerPidConfig);
            case "talonfx", "kraken" -> new TalonFXSwerveModule(index, config, drivePidConfig, steerPidConfig);
            default -> throw new IllegalArgumentException("Unknown module type: " + moduleType +
                ". Valid options: SparkMax, TalonFX");
        };
    }
}
