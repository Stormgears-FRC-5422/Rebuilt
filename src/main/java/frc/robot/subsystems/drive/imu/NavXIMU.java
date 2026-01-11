package frc.robot.subsystems.drive.imu;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;

/**
 * NavX IMU implementation using Studica AHRS.
 * Communicates via MXP SPI interface.
 */
public class NavXIMU implements IMU {
    private final AHRS navx;

    public NavXIMU() {
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
        navx.zeroYaw();
        System.out.println("NavX IMU initialized on MXP SPI");
    }

    @Override
    public Rotation2d getRotation() {
        return new Rotation2d(Degrees.of(-navx.getAngle()));
    }

    @Override
    public void reset() {
        navx.zeroYaw();
    }

    @Override
    public double getYaw() {
        return navx.getYaw();
    }
}
