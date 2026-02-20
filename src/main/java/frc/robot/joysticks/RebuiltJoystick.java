package frc.robot.joysticks;

import frc.utils.joysticks.DriveJoystick;

public abstract class RebuiltJoystick {
    protected final DriveJoystick driveJoystick;

    protected RebuiltJoystick(DriveJoystick driveJoystick) {
        this.driveJoystick = driveJoystick;
    }

    // Core driving - implemented once using the DriveJoystick interface
    public double getWpiX() {
        return driveJoystick.getWpiXSpeed();
    }

    public double getWpiY() {
        return driveJoystick.getWpiYSpeed();
    }

    public double getOmegaSpeed() {
        return driveJoystick.getOmegaSpeed();
    }

    public boolean getIntakeRun() {
        return true;
    }

    public double getTurbo() {
        return 0.0;
    }

    public boolean resetGyro() {
        return false;
    }

    public boolean wheelRadiusCharacterization() {
        return false;
    }

    // Logitech-specific (slider only) - safe default for Xbox
    public double getSlider() {
        return 0.0;
    }

    public double getLeftTriggerAxis() {
        return 0.0;
    }
    public boolean shooterIsHeldManual() {
        return false;
    }

    public boolean intakeTogglePressed() {
        return false;
    }
    public boolean getRobotRelative() {
        return true;
    }
    public boolean enableRumble() {
        return false;
    }
}
