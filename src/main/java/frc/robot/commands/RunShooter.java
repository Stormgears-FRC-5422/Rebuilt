package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.KrakenShooter;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

public class RunShooter extends StormCommand {
    private final KrakenShooter shooter;
    private final DoubleSupplier targetVelocityRpsgiver;

    public RunShooter(KrakenShooter shooter, DoubleSupplier targetVelocityRpsgiver) {
        this.shooter = shooter;
        this.targetVelocityRpsgiver = targetVelocityRpsgiver;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double targetRps = targetVelocityRpsgiver.getAsDouble();
        
        shooter.setTargetVelocity(targetRps);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
