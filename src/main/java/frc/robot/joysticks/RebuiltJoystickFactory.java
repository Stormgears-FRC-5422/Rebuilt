package frc.robot.joysticks;

public class RebuiltJoystickFactory {

    public static RebuiltJoystick getInstance(String joystickType, int port) throws IllegalJoystickTypeException {
        RebuiltJoystick instance;
        System.out.println("Initializing " + joystickType + " as Joystick");
        switch (joystickType.toLowerCase()) {
            case "xboxcontroller" -> instance = new RebuiltXboxController(port);
            case "logitechcontroller" -> instance = new RebuiltLogitechJoystick(port);
            default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
        }
        return instance;
    }
}
