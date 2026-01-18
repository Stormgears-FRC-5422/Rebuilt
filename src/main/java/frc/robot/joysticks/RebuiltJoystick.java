package frc.robot.joysticks;

public abstract class RebuiltJoystick {
    public double getWpiX() {
        return 0;
    }

    public double getWpiY() {
        return 0;
    }

    public double getOmegaSpeed() {
        return 0;
    }

    public boolean getRobotRelative() {
        return true;
    }

    public double getTurbo() {
        return 0.0;
    }

    public boolean zeroGyro() {
        return false;
    }

    public boolean zeroWheels() {
        return false;
    }

    public boolean holdSwerveState() {
        return false;
    }

    public boolean wheelRadiusCharacterization() {
        return false;
    }
}
