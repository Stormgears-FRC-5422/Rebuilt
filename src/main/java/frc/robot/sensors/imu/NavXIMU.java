package frc.robot.sensors.imu;

import com.studica.frc.AHRS;
import frc.robot.Constants;

/**
 * NavX IMU implementation using Studica AHRS.
 * Communicates via MXP SPI interface.
 */
public class NavXIMU extends IMUBase {
    private final AHRS navx;
    private final boolean ownsHardware;
    private double yawOffset = 0.0;
    private double simAngleDegrees = 0.0;
    private final Object simLock = new Object(); // Synchronize sim angle access between main and Notifier threads

    /**
     * Creates a new NavXIMU using default MXP SPI configuration.
     * This constructor creates and owns the NavX hardware.
     */
    public NavXIMU() {
        super("NavX");
        this.ownsHardware = true;
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
        navx.zeroYaw();
        console("Initialized on MXP SPI (inverted: " + Constants.Imu.inverted + ")");
    }

    /**
     * Creates a NavXIMU wrapper around an existing AHRS.
     * Use this when another library creates the NavX.
     * This constructor does not reset or reconfigure the hardware.
     *
     * @param existingNavX The AHRS instance to wrap
     */
    public NavXIMU(AHRS existingNavX) {
        super("NavX");
        this.ownsHardware = false;
        this.navx = existingNavX;
        console("Wrapping existing NavX (inverted: " + Constants.Imu.inverted + ")");
    }

    @Override
    protected double getRawAngle() {
        if (isSimulation) {
            synchronized (simLock) {
                return simAngleDegrees;
            }
        }
        return navx.getAngle() + yawOffset;
    }

    @Override
    public void reset() {
        navx.zeroYaw();
        yawOffset = 0.0;
        if (isSimulation) {
            simAngleDegrees = 0.0;
        }
    }

    @Override
    public void setYaw(double angleDegrees) {
        // NavX doesn't support setting yaw directly, so store offset
        if (isSimulation) {
            synchronized (simLock) {
                // Account for inversion: getRotation() multiplies by invertMultiplier,
                // so we need to store the inverse to get the desired angle back
                simAngleDegrees = angleDegrees * invertMultiplier;
            }
        } else {
            yawOffset = angleDegrees - navx.getAngle();
        }
    }

    @Override
    public void updateSimulation(double deltaAngleRadians) {
        if (isSimulation) {
            synchronized (simLock) {
                // Apply inversion to the raw angle update (invertMultiplier is applied on read in getRotation())
                simAngleDegrees += Math.toDegrees(deltaAngleRadians) * invertMultiplier;
            }
        }
    }
}
