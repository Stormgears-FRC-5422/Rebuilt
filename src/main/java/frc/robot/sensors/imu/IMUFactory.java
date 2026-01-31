package frc.robot.sensors.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants;

/**
 * Factory for creating IMU instances based on configuration.
 * Uses Constants.Imu.type to determine which implementation to create.
 * Also provides wrap() methods for wrapping externally-created hardware.
 */
public class IMUFactory {
    /**
     * Creates an IMU based on the configured type in Constants.Imu.type.
     */
    public static IMU createIMU() {
        String imuType = Constants.Imu.type;
        System.out.println("Creating IMU of type: " + imuType);

        return switch (imuType.toLowerCase()) {
            case "navx" -> new NavXIMU();
            case "pigeon2" -> new Pigeon2IMU();
            case "adxrs450" -> new ADXRS450IMU();
            default -> throw new IllegalArgumentException("Unknown IMU type: " + imuType +
                ". Valid options: NavX, Pigeon2, ADXRS450");
        };
    }

    /**
     * Wraps an existing Pigeon2 in our IMU interface.
     * Use this when another library (e.g., CTRE Swerve) creates the Pigeon2.
     */
    public static IMU wrap(Pigeon2 pigeon) {
        return new Pigeon2IMU(pigeon);
    }

    /**
     * Wraps an existing NavX (AHRS) in our IMU interface.
     */
    public static IMU wrap(AHRS navx) {
        return new NavXIMU(navx);
    }

    /**
     * Wraps an existing ADXRS450 gyro in our IMU interface.
     */
    public static IMU wrap(ADXRS450_Gyro gyro) {
        return new ADXRS450IMU(gyro);
    }
}
