package frc.robot.subsystems.drive.imu;

import frc.robot.Constants;

/**
 * Factory for creating IMU instances based on configuration.
 * Uses Constants.Drive.imuType to determine which implementation to create.
 */
public class IMUFactory {
    public static IMU createIMU() {
        String imuType = Constants.Drive.imuType;
        System.out.println("Creating IMU of type: " + imuType);

        return switch (imuType.toLowerCase()) {
            case "navx" -> new NavXIMU();
            case "pigeon2" -> new Pigeon2IMU();
            case "adxrs450" -> new ADXRS450IMU();
            default -> throw new IllegalArgumentException("Unknown IMU type: " + imuType +
                ". Valid options: NavX, Pigeon2, ADXRS450");
        };
    }
}
