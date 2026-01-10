package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Drive;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public abstract class DrivetrainBase extends StormSubsystem {

    public static boolean driveFlip = true;
    public static boolean fieldRelativeOn = true;
    protected double m_maxVelocityMetersPerSecond = 1;
    protected double m_maxAngularVelocityRadiansPerSecond = 1;
    protected double m_driveSpeedScale = 0;
    protected boolean m_fieldRelative = false;

    @AutoLogOutput
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        driveFlip = false;
        fieldRelativeOn = true;
    }   

    protected final void setDriveFlip(boolean flip) {
        driveFlip = flip;
    }

    protected final void setFieldRelativeOn(boolean flip) {
        fieldRelativeOn = flip;
    }

    protected final void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
        console("MaxDriveVelocity: " + m_maxVelocityMetersPerSecond);
        console("MaxAngularVelocity: " + m_maxAngularVelocityRadiansPerSecond);
    }

    public final void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, Drive.driveSpeedScale);
    }

    // Be careful scaling ChassisSpeeds. Need to scale X and Y the same or your robot will move in the wrong direction!
    public ChassisSpeeds scaleChassisSpeeds(ChassisSpeeds speeds, double scale) {
        return new ChassisSpeeds(scale * speeds.vxMetersPerSecond,
                scale * speeds.vyMetersPerSecond,
                scale * speeds.omegaRadiansPerSecond);
    }

    /**
     * Command the robot to drive. This method expects real speeds in
     * meters/second. Speed may be limited by speedScale and / or slew rate
     * limiter
     *
     * @param speeds Chassis speedsSwerveDriveConfiguration for the swerve, esp
     * from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds, fieldRelative, m_driveSpeedScale);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        if (fieldRelativeOn && fieldRelative) {
            Rotation2d rotation = getRotation();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
        } else {
            m_chassisSpeeds = speeds;
        }
        m_chassisSpeeds = scaleChassisSpeeds(m_chassisSpeeds, speedScale);
    }

    /**
     * Command the robot to drive, especially from Joystick This method expects
     * units from -1 to 1, and then scales them to the max speeds You should
     * call setMaxVelocities() before calling this method
     *
     * @param speeds Chassis speeds, especially from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void percentOutputDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
                fieldRelative);
    }

    public Rotation2d getRotation() {
        return new Rotation2d();
    }

    // Teach the drive that the current orientation is facing the opposite end of the field
    // this function is ideally alliance aware, and manages the pose and gyro as needed
    public void resetOrientation() {
    }

}
