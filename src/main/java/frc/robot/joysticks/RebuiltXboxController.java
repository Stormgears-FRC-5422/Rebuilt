package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class RebuiltXboxController extends RebuiltJoystick {
    private final StormXboxController xboxController;

    RebuiltXboxController(int port) {
        super(new StormXboxController(port));
        this.xboxController = (StormXboxController) driveJoystick;
    }

    // Core driving inherited from base class via DriveJoystick

    @Override
    public boolean getIntakeRun() {
        return xboxController.getLeftTrigger() > 0.2;
    }

    @Override
    public double getTurbo() {
        return xboxController.getRightTrigger();
    }

    @Override
    public boolean resetGyro() {
        return xboxController.getBackIsPressed();
    }

    @Override
    public boolean narrow() {
        return xboxController.getYButtonIsHeld();
    }

    @Override
    public boolean safe() {
        return xboxController.getYButtonIsPressed();
    }
    @Override
    public boolean autoShoot() {
        return xboxController.getAButtonIsHeld();
    }

    @Override
    public boolean intakeHome() {
        return xboxController.getLeftBumperIsPressed();
    }

    // getSlider() uses base default (0.0)
    // @Override
    // public double getLeftTriggerAxis() {
    //     return xboxController.getLeftTrigger();
    // }
    @Override
    public boolean getRobotRelative() {
        return xboxController.getLeftBumperIsPressed();
    }
    @Override
    public boolean intakeTogglePressed() {
        return xboxController.getXisPressed();
    }
    @Override
    public boolean enableRumble() {
        return xboxController.getRightBumperIsPressed();
    }
    @Override
    public boolean shootToAlliance() {
        return xboxController.getBButtonIsHeld();
    }

    @Override
    public boolean manualShoot() {
        return xboxController.getBButtonIsHeld();
    }

    @Override
    public boolean funnelFullPower() {
        return xboxController.getYButtonIsHeld();
    }

    @Override
    public boolean outtake() {
        return xboxController.getRightTrigger() > 0.2;
    }

    @Override
    public boolean intakeUp() {
        return xboxController.getLeftJoystickY() > frc.robot.Constants.ButtonBoard.intakeStickThreshold;
    }

    @Override
    public boolean intakeDown() {
        return xboxController.getLeftJoystickY() < -frc.robot.Constants.ButtonBoard.intakeStickThreshold;
    }
    @Override
    public boolean intakeMoved() {
        return Math.abs(xboxController.getLeftJoystickY()) > frc.robot.Constants.ButtonBoard.intakeStickThreshold;
    }

    @Override
    public boolean climbHome()       { return xboxController.getBackIsPressed(); }
    @Override
    public boolean climbSequence()   { return xboxController.getRightBumperIsPressed(); }
    @Override
    public boolean climbManualUp()   { return xboxController.getUpArrowPressed(); }
    @Override
    public boolean climbManualDown() { return xboxController.getDownArrowPressed(); }

    @Override
    public boolean wonAutoOverride()  { return xboxController.getRightArrowPressed(); }
    @Override
    public boolean lostAutoOverride() { return xboxController.getLeftArrowPressed(); }

    @Override
    public void setRumble(double strength) {
        xboxController.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, strength);
    }

    @Override
    public boolean testAutoShoot() {
        return xboxController.getPOV() == 0; // D-pad up
    }

    @Override
    public boolean wheelRadiusCharacterization() {
        return xboxController.getRotateIsPressed();
    }
}
