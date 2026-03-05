package frc.robot.sensors.imu;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;

/**
 * Pigeon2 IMU implementation using CTRE Phoenix6.
 * Can be configured to use CANivore bus.
 */
public class Pigeon2IMU extends IMUBase {
    private final Pigeon2 pigeon;
    @SuppressWarnings("unused")
    private final boolean ownsHardware;
    private double simAngleDegrees = 0.0;
    private final Object simLock = new Object(); // Synchronize sim angle access between main and Notifier threads

    /**
     * Creates a new Pigeon2IMU using configuration from Constants.
     * This constructor creates and owns the Pigeon2 hardware.
     */
    public Pigeon2IMU() {
        super("Pigeon2");
        this.ownsHardware = true;

        // Pigeon2 ID from constants, on CANivore bus if configured
        int pigeonID = Constants.Imu.pigeonID;
        String canBusName = Constants.Imu.canBusName;
        String busName = (canBusName == null || canBusName.isEmpty()) ? "" : canBusName;
        CANBus canBus = new CANBus(busName);

        pigeon = new Pigeon2(pigeonID, canBus);

        // Apply default configuration and verify it succeeded
        Pigeon2Configuration config = new Pigeon2Configuration();
        StatusCode status = pigeon.getConfigurator().apply(config, 0.050);
        if (status.isOK()) {
            console("Configured successfully (ID: " + pigeonID + ", Bus: " + busName + ", inverted: " + Constants.Imu.inverted + ")");
        } else {
            console("Configuration failed (ID: " + pigeonID + ", Bus: " + busName + "): " + status.toString());
        }

        pigeon.reset();
    }

    /**
     * Creates a Pigeon2IMU wrapper around an existing Pigeon2.
     * Use this when another library (e.g., CTRE Swerve) creates the Pigeon2.
     * This constructor does not reset or reconfigure the hardware.
     *
     * @param existingPigeon The Pigeon2 instance to wrap
     */
    public Pigeon2IMU(Pigeon2 existingPigeon) {
        super("Pigeon2");
        this.ownsHardware = false;
        this.pigeon = existingPigeon;
        console("Wrapping existing Pigeon2 (ID: " + existingPigeon.getDeviceID() +
                ", inverted: " + Constants.Imu.inverted + ")");
    }

    @Override
    protected double getRawAngle() {
        if (isSimulation) {
            synchronized (simLock) {
                return simAngleDegrees;
            }
        }
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public void reset() {
        pigeon.reset();
        if (isSimulation) {
            simAngleDegrees = 0.0;
        }
    }

    @Override
    public void setYaw(double angleDegrees) {
        pigeon.getConfigurator().setYaw(angleDegrees);
        if (isSimulation) {
            synchronized (simLock) {
                // Account for inversion: getRotation() multiplies by invertMultiplier,
                // so we need to store the inverse to get the desired angle back
                simAngleDegrees = angleDegrees * invertMultiplier;
                pigeon.getSimState().setRawYaw(simAngleDegrees);
            }
        }
    }

    @Override
    public void updateSimulation(double deltaAngleRadians) {
        if (isSimulation) {
            synchronized (simLock) {
                // Apply inversion to the raw angle update (invertMultiplier is applied on read in getRotation())
                simAngleDegrees += Math.toDegrees(deltaAngleRadians) * invertMultiplier;
                pigeon.getSimState().setRawYaw(simAngleDegrees);
            }
        }
    }
}
