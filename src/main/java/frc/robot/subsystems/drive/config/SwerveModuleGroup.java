package frc.robot.subsystems.drive.config;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveModuleGroup {

    // Use constants from RebuiltModuleConstants
    final int FRONT_LEFT = SwerveModuleConstants.FRONT_LEFT;
    final int FRONT_RIGHT = SwerveModuleConstants.FRONT_RIGHT;
    final int BACK_RIGHT = SwerveModuleConstants.BACK_RIGHT;
    final int BACK_LEFT = SwerveModuleConstants.BACK_LEFT;
    final int NUM_MODULES = SwerveModuleConstants.NUM_MODULES;

    final SwerveModule[] swerveModules;
    final SparkMax[] driveArray;
    final SparkMax[] steerArray;
    final CANcoder[] encoderArray;
    final PidConfig drivePidConfig;
    final PidConfig steerPidConfig;

    public SwerveModuleGroup() {
        ModuleConfig[] moduleConfigs = SwerveModuleConstants.getOrderedModuleConfigs();

        drivePidConfig = SwerveModuleConstants.drivePidConfig;
        steerPidConfig = SwerveModuleConstants.steerPidConfig;

        swerveModules = new SwerveModule[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            swerveModules[i] = new SwerveModule(
                    i,
                    moduleConfigs[i],
                    drivePidConfig,
                    steerPidConfig);
        }

        driveArray = new SparkMax[NUM_MODULES];
        steerArray = new SparkMax[NUM_MODULES];
        encoderArray = new CANcoder[NUM_MODULES];

        // Initialize arrays in module order
        for (int i = 0; i < NUM_MODULES; i++) {
            driveArray[i] = swerveModules[i].driveMotor;
            steerArray[i] = swerveModules[i].steerMotor;
            encoderArray[i] = swerveModules[i].steerCANCoder;
        }
    }

    public void periodic() {
        for (SwerveModule module : swerveModules) {
            module.periodic();
        }

        // The above loop updates the modules based on the pidConfig if dirty.
        // We need all the modules to see the dirty setting, so here we mark that as done
        if (drivePidConfig.isDirty()) {
            drivePidConfig.clearDirty();
            System.out.println(drivePidConfig);
        }

        if (steerPidConfig.isDirty()) {
            steerPidConfig.clearDirty();
            System.out.println(steerPidConfig);
        }
    }

    public Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
            swerveModules[FRONT_LEFT].config.offset,
            swerveModules[FRONT_RIGHT].config.offset,
            swerveModules[BACK_RIGHT].config.offset,
            swerveModules[BACK_LEFT].config.offset
        };
    }

    public SparkMax[] getDriveMotors() {
        return driveArray;
    }

    public SparkMax[] getSteerMotors() {
        return steerArray;
    }

    public CANcoder[] getEncoders() {
        return encoderArray;
    }

    public SwerveModule[] getModules() {
        return swerveModules;
    }

    public LinearVelocity getMaxLinearVelocity() {
        // return min drive velocity across all modules since the robot can only go as fast as its slowest module
        LinearVelocity minVelocity = MetersPerSecond.of(Double.POSITIVE_INFINITY);
        for (SwerveModule module : swerveModules) {
            LinearVelocity velocity = module.getMaxLinearVelocity();
            if (velocity.in(MetersPerSecond) < minVelocity.in(MetersPerSecond)) {
                minVelocity = velocity;
            }
        }
        return minVelocity;
    }

    public AngularVelocity getMaxAngularVelocity() {
        // The robot's maximum angular velocity is determined by how fast it can drive its modules
        // in a circle around the robot's center. We can use any module's offset since they should
        // all be equidistant from the center.
        double moduleRadius = swerveModules[FRONT_LEFT].config.offset.getNorm();
        double maxRadiansPerSecond = getMaxLinearVelocity().in(MetersPerSecond) / moduleRadius;
        return RadiansPerSecond.of(maxRadiansPerSecond);
    }

    public void setSwerveModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[] states) {
        if (states.length != NUM_MODULES) {
            throw new IllegalArgumentException("Number of module states must match number of modules");
        }

        for (int i = 0; i < NUM_MODULES; i++) {
            swerveModules[i].setSwerveModuleState(states[i]);
        }
    }

    public void stop() {
        for (SwerveModule m : swerveModules) {
            m.stop();
        }
    }
}
