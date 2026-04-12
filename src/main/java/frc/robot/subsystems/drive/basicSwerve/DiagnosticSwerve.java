package frc.robot.subsystems.drive.basicSwerve;

import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.basicSwerve.config.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Diagnostic swerve drive for low-level motor testing.
 * Works with any hardware type (SparkMax or TalonFX).
 * Provides direct voltage control and telemetry logging.
 */
public class DiagnosticSwerve extends DrivetrainBase {

    final SwerveModuleGroup moduleGroup;
    final SwerveModuleBase[] modules;

    final int FRONT_LEFT = SwerveModuleConstants.FRONT_LEFT;
    final int FRONT_RIGHT = SwerveModuleConstants.FRONT_RIGHT;
    final int BACK_RIGHT = SwerveModuleConstants.BACK_RIGHT;
    final int BACK_LEFT = SwerveModuleConstants.BACK_LEFT;

    public DiagnosticSwerve() {
        super();

        moduleGroup = new SwerveModuleGroup();
        setMaxVelocities(moduleGroup.getMaxLinearVelocity(),
                moduleGroup.getMaxAngularVelocity());

        modules = moduleGroup.getModules();
    }

    @Override
    public double getYawRateDegPerSec() {
        return 0.0; // Diagnostic-only drivetrain, no real gyro
    }

    @Override
    public void periodic() {
        double drivePercent = m_chassisSpeeds.vxMetersPerSecond * m_driveSpeedScale;
        double steerPercent = m_chassisSpeeds.omegaRadiansPerSecond * m_driveSpeedScale;

        // Set motor percent outputs using open-loop control
        for (SwerveModuleBase m : modules) {
            m.setMotors(drivePercent, steerPercent);
            m.periodic();  // Update sensors and telemetry
        }

        // DiagnosticSwerve-specific telemetry
        Logger.recordOutput("Drive/Diagnostic/DrivePercentCommand", drivePercent);
        Logger.recordOutput("Drive/Diagnostic/SteerPercentCommand", steerPercent);
    }
}
