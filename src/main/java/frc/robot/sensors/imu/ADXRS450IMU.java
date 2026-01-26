package frc.robot.sensors.imu;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants;

/**
 * ADXRS450 IMU implementation using WPILib.
 * Communicates via SPI interface on the RoboRIO.
 */
public class ADXRS450IMU extends IMUBase {
    private final ADXRS450_Gyro gyro;
    private final boolean ownsHardware;
    private double yawOffset = 0.0;
    private double simAngleDegrees = 0.0;
    private final Object simLock = new Object(); // Synchronize sim angle access between main and Notifier threads

    /**
     * Creates a new ADXRS450IMU using default SPI configuration.
     * This constructor creates and owns the gyro hardware.
     */
    public ADXRS450IMU() {
        super("ADXRS450");
        this.ownsHardware = true;
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        gyro.reset();
        console("Initialized on SPI (inverted: " + Constants.Imu.inverted + ")");
    }

    /**
     * Creates an ADXRS450IMU wrapper around an existing gyro.
     * Use this when another library creates the gyro.
     * This constructor does not calibrate or reset the hardware.
     *
     * @param existingGyro The ADXRS450_Gyro instance to wrap
     */
    public ADXRS450IMU(ADXRS450_Gyro existingGyro) {
        super("ADXRS450");
        this.ownsHardware = false;
        this.gyro = existingGyro;
        console("Wrapping existing ADXRS450 (inverted: " + Constants.Imu.inverted + ")");
    }

    @Override
    protected double getRawAngle() {
        if (isSimulation) {
            synchronized (simLock) {
                return simAngleDegrees;
            }
        }
        return gyro.getAngle() + yawOffset;
    }

    @Override
    public void reset() {
        gyro.reset();
        yawOffset = 0.0;
        if (isSimulation) {
            simAngleDegrees = 0.0;
        }
    }

    @Override
    public void setYaw(double angleDegrees) {
        // ADXRS450 doesn't support setting yaw directly, so store offset
        if (isSimulation) {
            synchronized (simLock) {
                // Account for inversion: getRotation() multiplies by invertMultiplier,
                // so we need to store the inverse to get the desired angle back
                simAngleDegrees = angleDegrees * invertMultiplier;
            }
        } else {
            yawOffset = angleDegrees - gyro.getAngle();
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
