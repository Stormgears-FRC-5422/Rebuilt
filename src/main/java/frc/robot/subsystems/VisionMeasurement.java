package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionMeasurement(
    Pose2d pose,
    double timestampSeconds,
    double closestTagDistanceMeters,
    int tagCount,
    String sourceLimelight
) {}
