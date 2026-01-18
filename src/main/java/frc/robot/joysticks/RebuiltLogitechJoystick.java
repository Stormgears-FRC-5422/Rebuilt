package frc.robot.joysticks;

import frc.utils.joysticks.StormLogitechController;

public class RebuiltLogitechJoystick extends RebuiltJoystick{

    StormLogitechController controller;

    public RebuiltLogitechJoystick(int port) {
        controller = new StormLogitechController(port);
    }

    public double getWpiX() {
        return controller.getWpiXSpeed();
    }

    public double getWpiY() {
        return controller.getWpiYSpeed();
    }

    public double getOmegaSpeed() {
        return controller.getOmegaSpeed();
    }

    public boolean shoot() {
        return controller.getRawButton(1);
    }

    public boolean outtake() {
        return controller.getRawButton(2);
    }

    public double getSlider() {
        return controller.getSliderAxis();
    }

    public boolean holdSwerveState() {
        return controller.getRawButton(7);
    }
}
