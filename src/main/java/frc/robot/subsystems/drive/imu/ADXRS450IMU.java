package frc.robot.subsystems.drive.imu;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants;

/**
 * ADXRS450 IMU implementation using WPILib.
 * Communicates via SPI interface on the RoboRIO.
 */
public class ADXRS450IMU extends IMUBase {
    private final ADXRS450_Gyro gyro;

    public ADXRS450IMU() {
        super("ADXRS450");
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        gyro.reset();
        console("Initialized on SPI (inverted: " + Constants.Drive.imuInverted + ")");
    }

    @Override
    protected double getRawAngle() {
        return gyro.getAngle();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
