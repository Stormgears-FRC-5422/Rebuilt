package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StormCommand extends Command {
    protected String _name = "StormCommand";
    protected int _count = 0;

    public StormCommand() {
        _name = getClass().getName();
        _name = _name.substring(_name.lastIndexOf('.') + 1);
        console("created");
    }

    @Override
    public void initialize() {
        // we want printing to alway happen on the first iteration. So
        _count = -1;
        console("initialized");
    }

    @Override public void execute() {
        _count++;
        console("running",500);
    }
    @Override
    public void end(boolean interrupted) {
        console("ended: interrupted = " + interrupted);
    }

    public void console(String message) {
        System.out.println(Timer.getFPGATimestamp() + ":Command " + _name + ": " + message);
    }
    public void console(String message, int iterations) {
        if (_count % iterations == 0) {
            console(message);
        }
    }
    public boolean safeAddRequirements(Subsystem... subsystems) {
        boolean unsafe = false;
        int counter = 0;
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) {
                console("arg "+ counter + " is null");
                unsafe = true;
            }else {
                addRequirements(subsystem);
            }
            counter++;
        }
        return unsafe;
    }
}
