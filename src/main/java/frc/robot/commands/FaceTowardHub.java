package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.utils.StormCommand;

public class FaceTowardHub extends StormCommand {

    VisionSubsystem visionSubsystem;
    public FaceTowardHub(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
