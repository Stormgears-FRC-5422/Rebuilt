package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoCommandFactory {

    private final AutoFactory autoFactory;

    public AutoCommandFactory(DrivetrainBase drivetrainBase) {

        autoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::declarePoseIsNow,
            drivetrainBase::followTrajectory,
            true,
            drivetrainBase
        );

        autoFactory.cache().loadTrajectory("Straight");
    }

    public Command getTheAuto() {
        return autoFactory.trajectoryCmd("Straight");
    }
}
