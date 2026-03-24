package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class EstimationManager extends StormSubsystem {

    private final DrivetrainBase drivetrain;
    private final PoseEstimator poseEstimator;
    private final VisionSubsystem vision;
    private final RobotState state;

    /** Vision measurement standard deviations [x(m), y(m), theta(rad)].
     *  theta is set very high because MegaTag2 already uses the gyro for heading. */
    private static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.7, 0.7, 9999999);

    public EstimationManager(DrivetrainBase drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.poseEstimator = drivetrain.getPoseEstimator();
        this.vision = vision;
        this.state = RobotState.getInstance();
    }

    /**
     * Returns the canonical PoseEstimator owned by this manager.
     */
    public PoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public void periodic() {
        super.periodic();

        // 1. Process odometry samples from drive
        ArrayList<OdometrySample> odometrySamples = drivetrain.getAndClearOdometrySamples();
        for (OdometrySample s : odometrySamples) {
            poseEstimator.updateOdometry(s.timestampSeconds(), s.gyroAngle(), s.modulePositions());
        }

        // 2. Integrate vision measurements
        if (vision != null) {
            ArrayList<VisionMeasurement> measurements = vision.getAndClearMeasurements();
            for (VisionMeasurement m : measurements) {
                if (Constants.Vision.mergeVision) {
                    Pose2d poseBefore = poseEstimator.getPose();

                    poseEstimator.addVisionMeasurement(
                        m.pose(),
                        m.timestampSeconds(),
                        VISION_STD_DEVS
                    );

                    Pose2d poseAfter = poseEstimator.getPose();
                    double correctionMeters = Math.hypot(
                        poseAfter.getX() - poseBefore.getX(),
                        poseAfter.getY() - poseBefore.getY());
                    double correctionDegrees = Math.abs(
                        poseAfter.getRotation().getDegrees() - poseBefore.getRotation().getDegrees());

                    String src = m.sourceLimelight();
                    Logger.recordOutput("Estimation/" + src + "/CorrectionMeters", correctionMeters);
                    Logger.recordOutput("Estimation/" + src + "/CorrectionDegrees", correctionDegrees);
                    Logger.recordOutput("Estimation/" + src + "/PoseBefore", poseBefore);
                    Logger.recordOutput("Estimation/" + src + "/PoseAfter", poseAfter);
                    Logger.recordOutput("Estimation/" + src + "/TagDistMeters", m.closestTagDistanceMeters());
                }
            }
        }

        // 3. Push canonical pose to RobotState
        Pose2d canonicalPose = poseEstimator.getPose();
        state.setPose(canonicalPose);
        Logger.recordOutput("Estimation/Pose", canonicalPose);
    }
}
