package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.drive.config.SwerveModuleConstants;
import frc.robot.subsystems.drive.config.SwerveModule;
import frc.robot.subsystems.drive.config.SwerveModuleGroup;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class DiagnosticSwerve extends DrivetrainBase {

    final double m_maxMotorVoltage = Drive.maxMotorVoltage;

    final SwerveModuleGroup moduleGroup;
    final SwerveModule[] modules;
    final SparkMax[] driveMotors;
    final SparkMax[] steerMotors;
    final CANcoder[] encoders;

    final int FRONT_LEFT = SwerveModuleConstants.FRONT_LEFT;
    final int FRONT_RIGHT = SwerveModuleConstants.FRONT_RIGHT;
    final int BACK_RIGHT = SwerveModuleConstants.BACK_RIGHT;
    final int BACK_LEFT = SwerveModuleConstants.BACK_LEFT;

    public DiagnosticSwerve() {
        super();

        moduleGroup = new SwerveModuleGroup();
        setMaxVelocities(moduleGroup.getMaxLinearVelocity().in(MetersPerSecond),
                moduleGroup.getMaxAngularVelocity().in(RadiansPerSecond));

        driveMotors = moduleGroup.getDriveMotors();
        steerMotors = moduleGroup.getSteerMotors();
        encoders = moduleGroup.getEncoders();
        modules = moduleGroup.getModules();
    }

    @Override
    public void periodic() {
        double driveSpeed = m_chassisSpeeds.vxMetersPerSecond;
        double steerSpeed = m_chassisSpeeds.omegaRadiansPerSecond;

        for (SwerveModule m : modules) {
            m.driveMotorSet(driveSpeed);
            m.steerMotorSet(steerSpeed);
        }

        Logger.recordOutput("fl steer", steerMotors[FRONT_LEFT].getEncoder().getPosition());
        Logger.recordOutput("fr steer", steerMotors[FRONT_RIGHT].getEncoder().getPosition());
        Logger.recordOutput("bl steer", steerMotors[BACK_LEFT].getEncoder().getPosition());
        Logger.recordOutput("br steer", steerMotors[BACK_RIGHT].getEncoder().getPosition());

        Logger.recordOutput("fl encoder", encoders[FRONT_LEFT].getPosition().getValue());
        Logger.recordOutput("fr encoder", encoders[FRONT_RIGHT].getPosition().getValue());
        Logger.recordOutput("bl encoder", encoders[BACK_LEFT].getPosition().getValue());
        Logger.recordOutput("br encoder", encoders[BACK_RIGHT].getPosition().getValue());

        Logger.recordOutput("fl drive pos", driveMotors[FRONT_LEFT].getEncoder().getPosition());
        Logger.recordOutput("fr drive pos", driveMotors[FRONT_RIGHT].getEncoder().getPosition());
        Logger.recordOutput("bl drive pos", driveMotors[BACK_LEFT].getEncoder().getPosition());
        Logger.recordOutput("br drive pos", driveMotors[BACK_RIGHT].getEncoder().getPosition());

        Logger.recordOutput("fl drive vel", driveMotors[FRONT_LEFT].getEncoder().getVelocity());
        Logger.recordOutput("fr drive vel", driveMotors[FRONT_RIGHT].getEncoder().getVelocity());
        Logger.recordOutput("bl drive vel", driveMotors[BACK_LEFT].getEncoder().getVelocity());
        Logger.recordOutput("br drive vel", driveMotors[BACK_RIGHT].getEncoder().getVelocity());
    }
}
