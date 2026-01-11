package frc.robot.subsystems.drive.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Degrees;

/**
 * Pigeon2 IMU implementation using CTRE Phoenix6.
 * Can be configured to use CANivore bus.
 */
public class Pigeon2IMU implements IMU {
    private final Pigeon2 pigeon;

    public Pigeon2IMU() {
        // Pigeon2 ID from constants, on CANivore bus if configured
        int pigeonID = Constants.Drive.pigeonID;
        String canBus = Constants.Drive.canBusName;

        pigeon = new Pigeon2(pigeonID, canBus);
        pigeon.reset();
        System.out.println("Pigeon2 IMU initialized (ID: " + pigeonID + ", Bus: " + canBus + ")");
    }

    @Override
    public Rotation2d getRotation() {
        // Pigeon2 uses getYaw() which returns a StatusSignal
        return new Rotation2d(Degrees.of(-pigeon.getYaw().getValueAsDouble()));
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }
}
