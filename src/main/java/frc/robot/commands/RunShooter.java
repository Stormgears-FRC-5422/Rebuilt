package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

public class RunShooter extends StormCommand {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetVelocityRpsgiver;

    public RunShooter(ShooterSubsystem shooter, DoubleSupplier targetVelocityRpsgiver) {
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
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
