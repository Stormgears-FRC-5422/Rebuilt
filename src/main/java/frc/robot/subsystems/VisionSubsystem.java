package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightHelpers;

import java.util.Optional;

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

    public double getDistFromTag(){
        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
        Pose2d poseV = poseEstimate.pose;
        Pose2d poseTag = FieldConstants.getPoseTag(getBestTag());
        Translation2d diff = poseV.minus(poseTag).getTranslation();
        double xDist = diff.getX();
        double yDist = diff.getY();
        return Math.sqrt(Math.pow(xDist,2)+ Math.pow(yDist,2));



    }

    public Transform2d getTurretOffset(double xDist1, double pivotDist, int pivotAngle, int planarAngle){
        double pAngle = Math.toRadians(pivotAngle);
        double plAngle = Math.toRadians(planarAngle);

        double plDist = xDist1 + pivotDist * Math.cos(pAngle);

        double xDist = plDist * Math.cos(plAngle);
        double yDist = plDist * Math.sin(plAngle);

        return new Transform2d(xDist,yDist, new Rotation2d(planarAngle));
    }



    @Override
    public void periodic() {
        super.periodic();

    }
}
