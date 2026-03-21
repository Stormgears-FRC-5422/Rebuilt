package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometrySample(
    double timestampSeconds,
    Rotation2d gyroAngle,
    SwerveModulePosition[] modulePositions
) {
    public OdometrySample {
        SwerveModulePosition[] copy = new SwerveModulePosition[modulePositions.length];
        for (int i = 0; i < modulePositions.length; i++) {
            copy[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, modulePositions[i].angle);
        }
        modulePositions = copy;
    }
}
