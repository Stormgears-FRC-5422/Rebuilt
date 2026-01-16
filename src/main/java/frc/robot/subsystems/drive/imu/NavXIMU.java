package frc.robot.subsystems.drive.imu;

import com.studica.frc.AHRS;
import frc.robot.Constants;

/**
 * NavX IMU implementation using Studica AHRS.
 * Communicates via MXP SPI interface.
 */
public class NavXIMU extends IMUBase {
    private final AHRS navx;

    public NavXIMU() {
        super("NavX");
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
        navx.zeroYaw();
        console("Initialized on MXP SPI (inverted: " + Constants.Drive.imuInverted + ")");
    }

    @Override
    protected double getRawAngle() {
        return navx.getAngle();
    }

    @Override
    public void reset() {
        navx.zeroYaw();
    }
}
