package frc.robot.subsystems.drive.basicSwerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.basicSwerve.config.ModuleConfig;
import frc.utils.pid.PidConfig;
import frc.robot.subsystems.drive.basicSwerve.config.SwerveModuleConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveModuleGroup {

    // Use constants from RebuiltModuleConstants
    final int FRONT_LEFT = SwerveModuleConstants.FRONT_LEFT;
    final int FRONT_RIGHT = SwerveModuleConstants.FRONT_RIGHT;
    final int BACK_RIGHT = SwerveModuleConstants.BACK_RIGHT;
    final int BACK_LEFT = SwerveModuleConstants.BACK_LEFT;
    final int NUM_MODULES = SwerveModuleConstants.NUM_MODULES;

    final SwerveModuleBase[] swerveModules;
    final PidConfig drivePidConfig;
    final PidConfig steerPidConfig;

    public SwerveModuleGroup() {
        ModuleConfig[] moduleConfigs = SwerveModuleConstants.getOrderedModuleConfigs();

        drivePidConfig = SwerveModuleConstants.drivePidConfig;
        steerPidConfig = SwerveModuleConstants.steerPidConfig;

        swerveModules = new SwerveModuleBase[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            swerveModules[i] = SwerveModuleFactory.createModule(
                    i,
                    moduleConfigs[i],
                    drivePidConfig,
                    steerPidConfig);
        }
    }

    public void periodic() {
        for (SwerveModuleBase module : swerveModules) {
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

    public SwerveModuleBase[] getModules() {
        return swerveModules;
    }

    public LinearVelocity getMaxLinearVelocity() {
        // return min drive velocity across all modules since the robot can only go as fast as its slowest module
        LinearVelocity minVelocity = MetersPerSecond.of(Double.POSITIVE_INFINITY);
        for (SwerveModuleBase module : swerveModules) {
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

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
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
        for (SwerveModuleBase m : swerveModules) {
            m.stop();
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void updateSimState(double deltaTime, double batteryVoltage) {
        for (SwerveModuleBase module : swerveModules) {
            module.updateSimState(deltaTime, batteryVoltage);
        }
    }
}
