package frc.robot.subsystems.misc;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.utils.StormSubsystem;
import frc.utils.motorcontrol.StormTalon;
import frc.robot.Constants.Music;

public class KrakenMusic extends StormSubsystem {
    private final Orchestra orchestra = new Orchestra();

    public KrakenMusic() {
        for (StormTalon motor : StormTalon.getRegistry().values()) {
            orchestra.addInstrument(motor);
        }
        System.out.println("KrakenMusic: loaded " + StormTalon.getRegistry().size() + " motors as instruments");

        var status = orchestra.loadMusic(Music.file);
        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        }
    }

    @Override
    public void periodic(){
        super.periodic();
        double voltage = RobotController.getBatteryVoltage();
        boolean isEnabled = DriverStation.isEnabled();

        double threshold;
        if (isEnabled) {
            threshold = Music.thresholdEnabled;
        } else {
            threshold = Music.thresholdDisabled;
        }

        if (voltage < threshold && !orchestra.isPlaying()) {
            orchestra.play();
        } else if (voltage >= threshold && orchestra.isPlaying()) {
            orchestra.stop();
        }
    }
    public void playR2D2() {
        orchestra.play();
    }

}

