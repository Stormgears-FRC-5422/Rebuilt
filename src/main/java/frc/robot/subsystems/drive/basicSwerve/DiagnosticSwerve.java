package frc.robot.subsystems.drive.basicSwerve;

import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.basicSwerve.config.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
        setMaxVelocities(moduleGroup.getMaxLinearVelocity().in(MetersPerSecond),
                moduleGroup.getMaxAngularVelocity().in(RadiansPerSecond));

        modules = moduleGroup.getModules();
    }

    @Override
    public void periodic() {
        double driveSpeed = m_chassisSpeeds.vxMetersPerSecond;
        double steerSpeed = m_chassisSpeeds.omegaRadiansPerSecond;

        // Set motor voltages using the hardware-agnostic method
        for (SwerveModuleBase m : modules) {
            m.driveMotorSet(driveSpeed);
            m.steerMotorSet(steerSpeed);
        }

        // Log telemetry using hardware-agnostic methods
        Logger.recordOutput("Diagnostic/" + modules[FRONT_LEFT].name + "/steerAngle", modules[FRONT_LEFT].getSteerAngle());
        Logger.recordOutput("Diagnostic/" + modules[FRONT_RIGHT].name + "/steerAngle", modules[FRONT_RIGHT].getSteerAngle());
        Logger.recordOutput("Diagnostic/" + modules[BACK_LEFT].name + "/steerAngle", modules[BACK_LEFT].getSteerAngle());
        Logger.recordOutput("Diagnostic/" + modules[BACK_RIGHT].name + "/steerAngle", modules[BACK_RIGHT].getSteerAngle());

        Logger.recordOutput("Diagnostic/" + modules[FRONT_LEFT].name + "/driveVelocity", modules[FRONT_LEFT].getDriveVelocity());
        Logger.recordOutput("Diagnostic/" + modules[FRONT_RIGHT].name + "/driveVelocity", modules[FRONT_RIGHT].getDriveVelocity());
        Logger.recordOutput("Diagnostic/" + modules[BACK_LEFT].name + "/driveVelocity", modules[BACK_LEFT].getDriveVelocity());
        Logger.recordOutput("Diagnostic/" + modules[BACK_RIGHT].name + "/driveVelocity", modules[BACK_RIGHT].getDriveVelocity());

        Logger.recordOutput("Diagnostic/driveSpeedCommand", driveSpeed);
        Logger.recordOutput("Diagnostic/steerSpeedCommand", steerSpeed);
    }
}
