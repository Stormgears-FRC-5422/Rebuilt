package frc.robot.sensors.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Null object pattern for when no pose estimation is available.
 * Returns identity pose and no-ops for all operations.
 */
public class NullPoseEstimator implements PoseEstimator {
    private static final Pose2d IDENTITY_POSE = new Pose2d();

    @Override
    public Pose2d getPose() {
        return IDENTITY_POSE;
    }

    @Override
    public void resetPose(Pose2d pose) {
        // No-op
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        // No-op
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        // No-op
    }
}
