package frc.robot.simulation.collision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Manages collision detection and visualization for robot simulation.
 * Handles initialization, periodic updates, pose constraints, and telemetry.
 */
public class CollisionManager {
    private final CollisionDetector detector;
    private Pose2d lastValidPose;
    private final boolean visualizationEnabled;

    // NetworkTables publishers for obstacle visualization (published to "Pose" table)
    private DoubleArrayPublisher fieldBoundaryPub;
    private DoubleArrayPublisher[] obstaclePubs;

    /**
     * Create a collision manager.
     * Only active in simulation mode when collision detection is enabled.
     * Automatically reads robot dimensions from Constants.Frame.
     *
     * @return CollisionManager instance, or null if not in simulation or disabled
     */
    public static CollisionManager createIfEnabled() {
        if (!RobotBase.isSimulation() || !Constants.Collision.enabled) {
            return null;
        }
        return new CollisionManager();
    }

    private CollisionManager() {
        // Calculate total dimensions including bumpers
        double robotLengthInches = Constants.Frame.lengthInches +
                                  2 * Constants.Frame.bumperThicknessInches;
        double robotWidthInches = Constants.Frame.widthInches +
                                 2 * Constants.Frame.bumperThicknessInches;

        this.detector = new CollisionDetector(robotLengthInches, robotWidthInches);
        this.lastValidPose = new Pose2d();
        this.visualizationEnabled = Constants.Collision.visualizeObstacles;

        if (visualizationEnabled) {
            // Publish obstacles to the same "Pose" NetworkTable as robot pose
            NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("Pose");
            fieldBoundaryPub = poseTable.getDoubleArrayTopic("FieldBoundary").publish();

            // Create publishers for each obstacle
            int obstacleCount = detector.getObstacleCount();
            obstaclePubs = new DoubleArrayPublisher[obstacleCount];
            for (int i = 0; i < obstacleCount; i++) {
                obstaclePubs[i] = poseTable.getDoubleArrayTopic("Obstacle/" + i).publish();
            }
        }

        System.out.println("Collision detection enabled for simulation (with bumpers)");
        System.out.println("  Robot collision bounds: " + robotLengthInches + "\" x " + robotWidthInches + "\"");
    }

    /**
     * Update collision detection and apply constraints to robot pose.
     * Call this every periodic cycle.
     *
     * @param currentPose Current desired robot pose
     * @param setPoseCallback Callback to update robot pose if constrained
     * @return The constrained pose (same as currentPose if no collision)
     */
    public Pose2d updateAndConstrain(Pose2d currentPose, PoseCallback setPoseCallback) {
        // Check for collision and constrain pose
        Pose2d constrainedPose = detector.constrainPose(currentPose, lastValidPose);

        // Update pose if constrained
        if (!constrainedPose.equals(currentPose)) {
            setPoseCallback.setPose(constrainedPose);

            // Log collision event
            Logger.recordOutput("Collision/Detected", true);
            Logger.recordOutput("Collision/DesiredPose", currentPose);
            Logger.recordOutput("Collision/ConstrainedPose", constrainedPose);
        } else {
            Logger.recordOutput("Collision/Detected", false);
        }

        lastValidPose = constrainedPose;

        // Update visualization
        if (visualizationEnabled) {
            updateVisualization(constrainedPose);
        }

        return constrainedPose;
    }

    /**
     * Update visualization telemetry.
     */
    private void updateVisualization(Pose2d currentPose) {
        // Publish obstacles to "Pose" NetworkTable (same as robot pose)
        Pose2d[] fieldBoundary = detector.getFieldBoundaryPoses();
        fieldBoundaryPub.set(posesToArray(fieldBoundary));

        for (int i = 0; i < obstaclePubs.length; i++) {
            Pose2d[] obstaclePoses = detector.getObstaclePoses(i);
            obstaclePubs[i].set(posesToArray(obstaclePoses));
        }

        // Log to AdvantageKit for replay
        Logger.recordOutput("Collision/Enabled", true);
        Logger.recordOutput("Collision/FieldBoundary", fieldBoundary);
        for (int i = 0; i < detector.getObstacleCount(); i++) {
            Logger.recordOutput("Collision/Obstacle/" + i, detector.getObstaclePoses(i));
        }
    }

    /** Convert Pose2d array to double array for NetworkTables. */
    private double[] posesToArray(Pose2d[] poses) {
        double[] array = new double[poses.length * 3];
        for (int i = 0; i < poses.length; i++) {
            array[i * 3] = poses[i].getX();
            array[i * 3 + 1] = poses[i].getY();
            array[i * 3 + 2] = poses[i].getRotation().getDegrees();
        }
        return array;
    }

    /**
     * Callback interface for updating robot pose.
     */
    @FunctionalInterface
    public interface PoseCallback {
        void setPose(Pose2d pose);
    }
}
