package frc.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StormSubsystem extends SubsystemBase {
    protected String _name = "StormSubsystem";
    protected int _count = 0;

    public StormSubsystem() {
        _name = getClass().getName();
        _name = _name.substring(_name.lastIndexOf('.') + 1);
        console("created");
    }

    @Override
    public void periodic() {
        _count++;
    }

    public void console(String message) {
        System.out.println("Subsystem " + _name + ": " + message);
    }

    public void console(String message, int iterations) {
        if (_count % iterations == 0) {
            console(message);
        }
    }
}
