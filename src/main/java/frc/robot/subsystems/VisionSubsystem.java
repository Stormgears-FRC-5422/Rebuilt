package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightHelpers;

public class VisionSubsystem extends StormSubsystem {

    SwerveDrivePoseEstimator poseEstimator;
    String limelightID;

    public VisionSubsystem(
//        SwerveDrivePoseEstimator poseEstimator
        String limelightID

    ){
//        this.poseEstimator = poseEstimator;
        this.limelightID = limelightID;
    }


    public boolean seesTag(){
        double ATid = LimelightHelpers.getFiducialID(limelightID);
        return (ATid >= 1);
    }

    public int getBestTag(){
        return (int) LimelightHelpers.getFiducialID(limelightID);
    }



    @Override
    public void periodic() {
        super.periodic();

    }
}
