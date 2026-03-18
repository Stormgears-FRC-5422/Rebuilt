package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometrySample(
    double timestampSeconds,
    Rotation2d gyroAngle,
    SwerveModulePosition[] modulePositions
) {}
