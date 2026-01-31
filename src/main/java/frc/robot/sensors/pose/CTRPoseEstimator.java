package frc.robot.sensors.pose;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Wraps CTRE's SwerveDrivetrain internal pose estimation for use with the PoseEstimator interface.
 * CTRE handles pose estimation internally via high-frequency odometry thread.
 * This wrapper just adapts the API - no update cycle management.
 */
public class CTRPoseEstimator implements PoseEstimator {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    /**
     * Creates a wrapper around CTRE's internal pose estimation.
     * @param drivetrain The CTRE SwerveDrivetrain instance
     */
    public CTRPoseEstimator(SwerveDrivetrain<?, ?, ?> drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        drivetrain.resetRotation(rotation);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }
}
