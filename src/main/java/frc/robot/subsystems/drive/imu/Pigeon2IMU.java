package frc.robot.subsystems.drive.imu;

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

    public Pigeon2IMU() {
        super("Pigeon2");

        // Pigeon2 ID from constants, on CANivore bus if configured
        int pigeonID = Constants.Drive.pigeonID;
        String canBusName = Constants.Drive.canBusName;
        String busName = (canBusName == null || canBusName.isEmpty()) ? "" : canBusName;
        CANBus canBus = new CANBus(busName);

        pigeon = new Pigeon2(pigeonID, canBus);

        // Apply default configuration and verify it succeeded
        Pigeon2Configuration config = new Pigeon2Configuration();
        StatusCode status = pigeon.getConfigurator().apply(config, 0.050);
        if (status.isOK()) {
            console("Configured successfully (ID: " + pigeonID + ", Bus: " + busName + ", inverted: " + Constants.Drive.imuInverted + ")");
        } else {
            console("Configuration failed (ID: " + pigeonID + ", Bus: " + busName + "): " + status.toString());
        }

        pigeon.reset();
    }

    @Override
    protected double getRawAngle() {
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public void reset() {
        pigeon.reset();
    }
}
