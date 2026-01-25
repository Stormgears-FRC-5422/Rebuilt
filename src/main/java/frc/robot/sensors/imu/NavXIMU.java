package frc.robot.sensors.imu;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.RobotBase;
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
            return simAngleDegrees;
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
            simAngleDegrees = angleDegrees;
        } else {
            yawOffset = angleDegrees - navx.getAngle();
        }
    }

    @Override
    public void updateSimulation(double deltaAngleRadians) {
        if (isSimulation) {
            // Apply inversion to the raw angle update (invertMultiplier is applied on read in getRotation())
            simAngleDegrees += Math.toDegrees(deltaAngleRadians) * invertMultiplier;
        }
    }
}
