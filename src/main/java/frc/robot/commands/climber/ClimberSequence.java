package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberSequence extends SequentialCommandGroup {
    public ClimberSequence(Climber climber) {
        addCommands(
            new InstantCommand(() -> climber.setState(Climber.ClimberState.READYTOCLIMB), climber)
                .andThen(Commands.waitUntil(() -> climber.atTargetDegrees(Constants.Climber.readyToClimbDegrees))),
            new InstantCommand(() -> climber.setState(Climber.ClimberState.CLIMBING), climber)
                .andThen(Commands.waitUntil(() -> climber.atTargetDegrees(Constants.Climber.climbedDegrees))),
            new InstantCommand(() -> climber.setState(Climber.ClimberState.CLIMBED), climber)
        );
    }
}
