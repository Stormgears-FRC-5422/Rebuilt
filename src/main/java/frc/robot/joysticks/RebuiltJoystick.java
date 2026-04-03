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

    public boolean narrow() {
        return false;
    }

    // Logitech-specific (slider only) - safe default for Xbox
    public double getSlider() {
        return 0.0;
    }

    public double getLeftTriggerAxis() {
        return 0.0;
    }
    public boolean autoShoot() {
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
    public boolean shootToAlliance() {
        return false;
    }
    public boolean manualShoot(){return false;}
    public boolean intakeUp(){return false;}
    public boolean intakeDown(){return false;}
    public boolean intakeMoved(){return false;}
    public boolean intakeHome(){return false;}

    public boolean climbHome()       { return false; }
    public boolean climbSequence()   { return false; }
    public boolean climbManualUp()   { return false; }
    public boolean climbManualDown() { return false; }

    // --- Game data overrides (joystick2 / operator) ---
    public boolean wonAutoOverride()   { return false; }
    public boolean lostAutoOverride()  { return false; }
    public boolean testAutoShoot()     { return false; }

    // --- Controller rumble ---
    public void setRumble(double strength) {}
    public boolean safe() {return false;}
    public boolean wheelRadiusCharacterization() {return false;}
}
