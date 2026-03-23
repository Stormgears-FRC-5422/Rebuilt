package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import frc.robot.subsystems.misc.BatteryMonitor;
import frc.robot.game.GameDataTracker;
import frc.robot.game.GameDataTracker.ShiftInfo;

import static edu.wpi.first.units.Units.*;

public class Lights extends StormSubsystem {

    private final RobotState m_robotState;

    Color RED_COLOR = Color.kRed;
    Color BLUE_COLOR = Color.kBlue;
    Color YELLOW_COLOR = Color.kYellow;
    Color GREEN_COLOR = Color.kGreen;
    Color ORANGE_COLOR = new Color(255, 32, 0);
    Color PINK_COLOR = new Color(225, 105, 180);
    Color PURPLE_COLOR = Color.kPurple;

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private AddressableLEDBufferView m_left;
    private AddressableLEDBufferView m_right;

    private boolean pulse = false;

    public Lights() {
        addressableLED = new AddressableLED(Constants.Lights.port);
        addressableLEDBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);

        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.start();
        setViews();
        m_robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        setBatteryPulse();
        switchShift();

        // Write the data to the LED strip
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    // Wrapper function to allow pulse effect to be applied to any pattern.
    // Call this function rather than calling pattern.applyTo()
    private <T extends LEDReader & LEDWriter> void patternApplyTo(LEDPattern basePattern, T view) {
        LEDPattern finalPattern = basePattern;

        if (pulse) {
            finalPattern = basePattern.breathe(Seconds.of(Constants.Lights.pulsePeriodSeconds));
        }

        finalPattern.applyTo(view);
    }

    public void setAllianceColor() {
        if (m_robotState.getAlliance() == RobotState.StateAlliance.RED) {
            setSolid(RED_COLOR);
        } else if (m_robotState.getAlliance() == RobotState.StateAlliance.BLUE) {
            setSolid(BLUE_COLOR);
        } else if (m_robotState.getAlliance() == RobotState.StateAlliance.MISSING) {
            setSolid(PURPLE_COLOR);
        }
    }

    public void setBatteryPulse() {
        if (m_robotState.getBatteryState() == BatteryMonitor.BatteryState.GOOD) {
            pulse = false;
        } else if (m_robotState.getBatteryState() == BatteryMonitor.BatteryState.WARN) {
            pulse = true;
        } else if (m_robotState.getBatteryState() == BatteryMonitor.BatteryState.BAD) {
            pulse = true;
        }
    }

    public void setRainbow(boolean forward) {
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        Distance kLedSpacing = Meters.of(1.0 / Constants.Lights.ledDensityPerMeter);
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(
                MetersPerSecond.of(Constants.Lights.scrollSpeedMPS), kLedSpacing);
        if (!forward)
            scrollingRainbow = scrollingRainbow.reversed();

        patternApplyTo(scrollingRainbow, m_left);
        patternApplyTo(scrollingRainbow, m_right);
    }

    public void setSolid(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        patternApplyTo(pattern, addressableLEDBuffer);
    }

    public void setViews() {
        var leftView = addressableLEDBuffer.createView(Constants.Lights.leftViewStart, Constants.Lights.leftViewEnd);
        m_left = Constants.Lights.leftViewReversed ? leftView.reversed() : leftView;
        var rightView = addressableLEDBuffer.createView(Constants.Lights.rightViewStart, Constants.Lights.rightViewEnd);
        m_right = Constants.Lights.rightViewReversed ? rightView.reversed() : rightView;
    }

    public void switchShift() {
        ShiftInfo info = GameDataTracker.getShiftInfo();

        switch (info.currentShift()) {
            case AUTO:
                setAllianceColor();
                break;
            case DISABLED:
                setSolid(ORANGE_COLOR);
                break;
            case ENDGAME:
                setRainbow(true);
                break;
            default:
                if (info.ourShift()) {
                    setSolid(GREEN_COLOR);
                } else {
                    setSolid(RED_COLOR);
                }
                if (info.remainingTime() < Constants.Lights.shiftWarningSeconds) {
                    setSolid(YELLOW_COLOR);
                }
                break;
        }
    }
}
