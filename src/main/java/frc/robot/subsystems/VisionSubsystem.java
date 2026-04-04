package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightHelpers;
import frc.utils.vision.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

public class VisionSubsystem extends StormSubsystem {

    private final String[] limelightIDs;

    /** Drivetrain reference — used for gyro heading (MegaTag2 requirement) */
    private DrivetrainBase drivetrain;

    /** Maximum closest-tag distance (meters) to accept an estimate, by tag count.
     *  Single-tag MegaTag2 estimates degrade rapidly with distance (54% >0.5m error
     *  at range in MAWOR Q1 logs), so they get a tighter cutoff. */
    private static final double MAX_DIST_SINGLE_TAG = Constants.Vision.maxTagDistSingleTag;
    private static final double MAX_DIST_MULTI_TAG = Constants.Vision.maxTagDistMultiTag;

    private ArrayList<VisionMeasurement> latestMeasurements = new ArrayList<>();

    // Per-cycle cached query results (updated in periodic)
    private boolean anyTagSeen;
    private double closestTagDistAnyLL = Double.MAX_VALUE;
    private int totalTagCount;

    public VisionSubsystem(
        DrivetrainBase drivetrain,
        String... limelightIDs
    ){
        this.limelightIDs = limelightIDs;
        this.maxTagsSeen = new int[limelightIDs.length];
        this.drivetrain = drivetrain;
    }

    /**
     * Returns all valid measurements collected since the last call and clears the internal list.
     * Consumed-on-read: each measurement is returned exactly once.
     */
    public ArrayList<VisionMeasurement> getAndClearMeasurements() {
        ArrayList<VisionMeasurement> measurements = latestMeasurements;
        latestMeasurements = new ArrayList<>();
        return measurements;
    }

    /** Returns true if any limelight currently sees at least one AprilTag. */
    public boolean seesAnyTag() { return anyTagSeen; }

    /** Returns the minimum tag distance (meters) across all limelights this cycle. */
    public double getClosestTagDistAnyLL() { return closestTagDistAnyLL; }

    /** Returns the number of unique tags seen across all limelights this cycle. */
    public int getTotalTagCount() { return totalTagCount; }

    public boolean seesTag(String limelightID) {
        double ATid = LimelightHelpers.getFiducialID(limelightID);
        return (ATid >= 1);
    }

    public int getBestTag(String limelightID) {
        return (int) LimelightHelpers.getFiducialID(limelightID);
    }

    /**
     * Returns the closest tag distance from a cached PoseEstimate.
     */
    private static double closestTagDist(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) {
            return Double.MAX_VALUE;
        }
        double minDist = Double.MAX_VALUE;
        for (LimelightHelpers.RawFiducial fid : estimate.rawFiducials) {
            if (fid != null && fid.distToRobot < minDist) {
                minDist = fid.distToRobot;
            }
        }
        return minDist;
    }

    private final int[] maxTagsSeen;

    private void logLLs(Map<String, PoseEstimate> estimates) {
        for (int llIdx = 0; llIdx < limelightIDs.length; llIdx++) {
            String ll = limelightIDs[llIdx];
            String start = "Vision/" + ll + "/";

            PoseEstimate estimate = estimates.get(ll);
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
            Logger.recordOutput(start + "ClosestTagDist", closestTagDist(estimate));

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
        if (drivetrain != null) {
            double robotYaw = drivetrain.getRotation().getDegrees();
            double yawRate = drivetrain.getYawRateDegPerSec();
            for (String ll : limelightIDs) {
                LimelightHelpers.SetRobotOrientation(ll, robotYaw, yawRate, 0, 0, 0, 0);
            }
        }

        // Fetch estimates once per limelight — reused by logging, closest-LL, and measurement collection
        Map<String, PoseEstimate> estimates = new HashMap<>();
        for (String ll : limelightIDs) {
            estimates.put(ll, LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll));
        }

        // Compute per-cycle aggregate stats
        anyTagSeen = false;
        closestTagDistAnyLL = Double.MAX_VALUE;
        HashSet<Integer> uniqueTags = new HashSet<>();
        for (String ll : limelightIDs) {
            PoseEstimate est = estimates.get(ll);
            if (est != null && est.tagCount > 0) {
                anyTagSeen = true;
                double dist = closestTagDist(est);
                if (dist < closestTagDistAnyLL) closestTagDistAnyLL = dist;
                for (LimelightHelpers.RawFiducial fid : est.rawFiducials) {
                    if (fid != null) uniqueTags.add(fid.id);
                }
            }
        }
        totalTagCount = uniqueTags.size();

        // Log per-limelight data
        logLLs(estimates);

        // Find closest LL
        String closestLL = null;
        double closestLLDist = Double.MAX_VALUE;
        for (String ll : limelightIDs) {
            if (seesTag(ll)) {
                double dist = closestTagDist(estimates.get(ll));
                if (dist < closestLLDist) {
                    closestLLDist = dist;
                    closestLL = ll;
                }
            }
        }
        if (closestLL == null && limelightIDs.length > 0) {
            closestLL = limelightIDs[0];
        }
        Logger.recordOutput("Vision/ActiveLL", closestLL);

        // Collect valid measurements from ALL limelights
        for (String ll : limelightIDs) {
            PoseEstimate estimate = estimates.get(ll);
            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }

            double tagDist = closestTagDist(estimate);
            double maxDist = estimate.tagCount >= 2 ? MAX_DIST_MULTI_TAG : MAX_DIST_SINGLE_TAG;
            if (tagDist > maxDist) {
                continue;
            }

            latestMeasurements.add(new VisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                tagDist,
                estimate.tagCount,
                ll
            ));
        }

        // Log closest-LL summary
        if (closestLL != null) {
            PoseEstimate closestEstimate = estimates.get(closestLL);
            if (closestEstimate != null && closestEstimate.tagCount > 0) {
                double dist = closestTagDist(closestEstimate);
                double maxDist = closestEstimate.tagCount >= 2 ? MAX_DIST_MULTI_TAG : MAX_DIST_SINGLE_TAG;
                boolean accepted = dist <= maxDist;
                Logger.recordOutput("Vision/Closest/HasTarget", true);
                Logger.recordOutput("Vision/Closest/Accepted", accepted);
                Logger.recordOutput("Vision/Closest/TagCount", closestEstimate.tagCount);
                Logger.recordOutput("Vision/Closest/Pose", closestEstimate.pose);
                Logger.recordOutput("Vision/Closest/AvgTagDist", closestEstimate.avgTagDist);
                Logger.recordOutput("Vision/Closest/ClosestTagDist", dist);
            } else {
                Logger.recordOutput("Vision/Closest/HasTarget", false);
                Logger.recordOutput("Vision/Closest/Accepted", false);
                Logger.recordOutput("Vision/Closest/TagCount", 0);
                Logger.recordOutput("Vision/Closest/Pose", new Pose2d());
                Logger.recordOutput("Vision/Closest/AvgTagDist", 0.0);
                Logger.recordOutput("Vision/Closest/ClosestTagDist", 0.0);
            }
        }
    }
}
