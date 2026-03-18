package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightHelpers;
import frc.utils.vision.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends StormSubsystem {

    String[] limelightIDs;

    /** Drivetrain reference — used for gyro heading and pose estimator */
    private DrivetrainBase drivetrain;
    private PoseEstimator drivetrainPoseEstimator;
    private boolean hasDrive = true;

    /** Maximum distance to accept a tag measurement, in feet */
    private static final double MAX_TAG_DISTANCE_FEET = 9.0;
    private static final double MAX_TAG_DISTANCE_METERS = MAX_TAG_DISTANCE_FEET * 0.3048;

    /** Vision measurement standard deviations [x(m), y(m), theta(rad)].
     *  theta is set very high because MegaTag2 already uses the gyro for heading. */
    private static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.7, 0.7, 9999999);


    public VisionSubsystem(
//        SwerveDrivePoseEstimator poseEstimator,
        DrivetrainBase drivetrain,
        String... limelightIDs
    ){
//        this.poseEstimator = poseEstimator;
        this.limelightIDs = limelightIDs;
        this.maxTagsSeen = new int[limelightIDs.length];
        if(drivetrain !=null){
            this.drivetrain = drivetrain;
            this.drivetrainPoseEstimator = drivetrain.getPoseEstimator();
            hasDrive = true;
        }else{
            hasDrive = false;
        }

    }


    public boolean seesTag(String limelightID) {
        double ATid = LimelightHelpers.getFiducialID(limelightID);
        return (ATid >= 1);
    }

    public int getBestTag(String limelightID) {
        return (int) LimelightHelpers.getFiducialID(limelightID);
    }

    public double getDistFromTag(String limelightID) {
        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return Double.MAX_VALUE;
        }

        // Use the closest individual tag distance measured directly by the camera.
        // Why not compute it from poseV.minus(poseTag)?
        //   The Limelight USES the tag to CALCULATE poseV. So if poseV is wrong,
        //   the distance you compute from it is also wrong — it's circular.
        //   rawFiducials[].distToRobot is an independent optical measurement
        //   that doesn't depend on the pose at all.
        double minDist = Double.MAX_VALUE;
        for (LimelightHelpers.RawFiducial fid : poseEstimate.rawFiducials) {
            if (fid != null && fid.distToRobot < minDist) {
                minDist = fid.distToRobot;
            }
        }
        return minDist;
    }

    public Transform2d getTurretOffset(double xDist1, double pivotDist, int pivotAngle, int planarAngle) {
        double pAngle = Math.toRadians(pivotAngle);
        double plAngle = Math.toRadians(planarAngle);

        double plDist = xDist1 + pivotDist * Math.cos(pAngle);

        double xDist = plDist * Math.cos(plAngle);
        double yDist = plDist * Math.sin(plAngle);

        return new Transform2d(xDist, yDist, new Rotation2d(planarAngle));
    }

    /** Returns true if any limelight sees at least one tag. */
    public boolean seesAnyTag() {
        for (String ll : limelightIDs) {
            if (seesTag(ll)) return true;
        }
        return false;
    }

    /** Returns the smallest closest-tag distance across all limelights. */
    public double getClosestTagDistAnyLL() {
        double minDist = Double.MAX_VALUE;
        for (String ll : limelightIDs) {
            double dist = getDistFromTag(ll);
            if (dist < minDist) {
                minDist = dist;
            }
        }
        return minDist;
    }

    /** Returns the number of unique tags seen across all limelights this cycle. */
    public int getTotalTagCount() {
        java.util.HashSet<Integer> tagIds = new java.util.HashSet<>();
        for (String ll : limelightIDs) {
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
            if (estimate != null && estimate.rawFiducials != null) {
                for (LimelightHelpers.RawFiducial fid : estimate.rawFiducials) {
                    if (fid != null) {
                        tagIds.add(fid.id);
                    }
                }
            }
        }
        return tagIds.size();
    }

    public void setAllLimelightOrientations(double robotYaw) {
        for (String ll : limelightIDs) {
            LimelightHelpers.SetRobotOrientation(ll, robotYaw, 0, 0, 0, 0, 0);
        }
    }

    private final int[] maxTagsSeen;

    public void logLLs(){
        for (int llIdx = 0; llIdx < limelightIDs.length; llIdx++) {
            String limelightID = limelightIDs[llIdx];
            String start = "Vision/" + limelightID + "/";

            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
            if (estimate == null || estimate.tagCount == 0) {
                Logger.recordOutput(start + "HasTarget", false);
                Logger.recordOutput(start + "TagCount", 0);
                Logger.recordOutput(start + "AvgTagDist", 0.0);
                Logger.recordOutput(start + "ClosestTagDist", 0.0);
                for (int i = 0; i < maxTagsSeen[llIdx]; i++) {
                    Logger.recordOutput(start + "Tags/Tag" + i + "_ID", 0);
                    Logger.recordOutput(start + "Tags/Tag" + i + "_DistMeters", 0.0);
                }
                continue;
            }

            if (estimate.rawFiducials.length > maxTagsSeen[llIdx]) {
                maxTagsSeen[llIdx] = estimate.rawFiducials.length;
            }

            Logger.recordOutput(start + "HasTarget", true);
            Logger.recordOutput(start + "TagCount", estimate.tagCount);
            Logger.recordOutput(start + "Pose", estimate.pose);
            Logger.recordOutput(start + "AvgTagDist", estimate.avgTagDist);
            Logger.recordOutput(start + "ClosestTagDist", getDistFromTag(limelightID));

            for (int i = 0; i < maxTagsSeen[llIdx]; i++) {
                if (i < estimate.rawFiducials.length && estimate.rawFiducials[i] != null) {
                    Logger.recordOutput(start + "Tags/Tag" + i + "_ID", estimate.rawFiducials[i].id);
                    Logger.recordOutput(start + "Tags/Tag" + i + "_DistMeters", estimate.rawFiducials[i].distToRobot);
                } else {
                    Logger.recordOutput(start + "Tags/Tag" + i + "_ID", 0);
                    Logger.recordOutput(start + "Tags/Tag" + i + "_DistMeters", 0.0);
                }
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        // MegaTag2 requires the robot orientation to be set each cycle for all limelights
        if(hasDrive){
            double robotYaw = drivetrain.getRotation().getDegrees();
            setAllLimelightOrientations(robotYaw);
        }

        logLLs();

        // Snapshot pose before any vision corrections this cycle
        Pose2d poseBefore = (hasDrive) ? drivetrainPoseEstimator.getPose() : new Pose2d();

        int acceptedCount = 0;

        for (String ll : limelightIDs) {
            String logPrefix = "Vision/" + ll + "/Merge/";

            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

            if (estimate == null || estimate.tagCount == 0) {
                Logger.recordOutput(logPrefix + "HasTarget", false);
                Logger.recordOutput(logPrefix + "Accepted", false);
                continue;
            }

            double closestTagDist = getDistFromTag(ll);
            boolean accepted = closestTagDist <= MAX_TAG_DISTANCE_METERS;

            Logger.recordOutput(logPrefix + "HasTarget", true);
            Logger.recordOutput(logPrefix + "Accepted", accepted);
            Logger.recordOutput(logPrefix + "TagCount", estimate.tagCount);
            Logger.recordOutput(logPrefix + "Pose", estimate.pose);
            Logger.recordOutput(logPrefix + "AvgTagDist", estimate.avgTagDist);
            Logger.recordOutput(logPrefix + "ClosestTagDist", closestTagDist);

            if (!accepted) continue;

            acceptedCount++;

            if (hasDrive && Constants.Vision.mergeVision) {
                drivetrainPoseEstimator.addVisionMeasurement(
                    estimate.pose,
                    estimate.timestampSeconds,
                    VISION_STD_DEVS
                );
            }
        }

        Logger.recordOutput("Vision/AcceptedCount", acceptedCount);

        if (hasDrive && Constants.Vision.mergeVision && acceptedCount > 0) {
            Pose2d poseAfter = drivetrainPoseEstimator.getPose();
            double dx = poseAfter.getX() - poseBefore.getX();
            double dy = poseAfter.getY() - poseBefore.getY();
            double correctionMeters = Math.sqrt(dx * dx + dy * dy);
            double correctionDegrees = Math.abs(
                poseAfter.getRotation().getDegrees() - poseBefore.getRotation().getDegrees()
            );

            Logger.recordOutput("Vision/CorrectionMeters", correctionMeters);
            Logger.recordOutput("Vision/CorrectionDegrees", correctionDegrees);
            Logger.recordOutput("Vision/PoseBefore", poseBefore);
            Logger.recordOutput("Vision/PoseAfter", poseAfter);
        }
    }
}
