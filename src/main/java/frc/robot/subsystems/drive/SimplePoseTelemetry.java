package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class SimplePoseTelemetry {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub;

    private final double[] poseArray = new double[3];
//    private int count = 0;

    public SimplePoseTelemetry(String name) {
        // Set the NetworkTables type for visualization in Glass
        fieldPub = table.getDoubleArrayTopic(name).publish();
        StringPublisher fieldTypePub = table.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");
    }

    /**
     * Publish the robot's pose to NetworkTables.
     *
     * @param pose The current position and orientation on the field of this thing
     */
    public void telemeterize(Pose2d pose) {
        /* Convert pose to an array format (X, Y, Rotation in degrees) */
        poseArray[0] = pose.getX();
        poseArray[1] = pose.getY();
        poseArray[2] = pose.getRotation().getDegrees();
        fieldPub.set(poseArray);
//        if (count++ % 100 == 0) {
//            System.out.println("Pose is " + pose);
//        }
    }
}
