package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;
import frc.utils.StormSubsystem;
import frc.robot.subsystems.misc.BatteryMonitor;

import static edu.wpi.first.units.Units.*;

public class Lights extends StormSubsystem {

    private final RobotState m_robotState;

    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer addressableLEDBuffer;
    private AddressableLEDBufferView m_bottom;
    private AddressableLEDBufferView m_top;

    private boolean pulse = false;
    private final double pulsePeriod = 5.0;

    // Pre-built patterns — created once, reused every loop to avoid GC pressure
    private final LEDPattern rainbowForward;
    private final LEDPattern rainbowReversed;
    private final LEDPattern patternRed    = LEDPattern.solid(Color.kRed);
    private final LEDPattern patternGreen  = LEDPattern.solid(Color.kGreen);
    private final LEDPattern patternBlue   = LEDPattern.solid(Color.kBlue);
    private final LEDPattern patternOrange = LEDPattern.solid(new Color(255, 32, 0));
    private final LEDPattern patternPurple = LEDPattern.solid(Color.kPurple);

    // Pre-built breathe variants — avoids creating wrapper objects on every loop tick
    private final LEDPattern rainbowForwardBreathe;
    private final LEDPattern rainbowReversedBreathe;
    private final LEDPattern patternRedBreathe;
    private final LEDPattern patternGreenBreathe;
    private final LEDPattern patternBlueBreathe;
    private final LEDPattern patternOrangeBreathe;
    private final LEDPattern patternPurpleBreathe;

    public Lights() {
        addressableLED = new AddressableLED(Constants.Lights.port);
        addressableLEDBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);

        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.start();
        setViews();

        LEDPattern rainbow = LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.45), Meters.of(1.0 / 120.0));
        rainbowForward  = rainbow;
        rainbowReversed = rainbow.reversed();

        var breathePeriod = Seconds.of(pulsePeriod);
        rainbowForwardBreathe  = rainbowForward.breathe(breathePeriod);
        rainbowReversedBreathe = rainbowReversed.breathe(breathePeriod);
        patternRedBreathe      = patternRed.breathe(breathePeriod);
        patternGreenBreathe    = patternGreen.breathe(breathePeriod);
        patternBlueBreathe     = patternBlue.breathe(breathePeriod);
        patternOrangeBreathe   = patternOrange.breathe(breathePeriod);
        patternPurpleBreathe   = patternPurple.breathe(breathePeriod);

        m_robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        setBatteryPulse();

        if (DriverStation.isDisabled()) {
            setRainbow(true);
        } else {
            switchLightState(m_robotState.getHopperState());
        }

        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    // Selects base or pre-built breathe variant — zero allocation
    private <T extends LEDReader & LEDWriter> void patternApplyTo(
            LEDPattern base, LEDPattern breatheVer, T view) {
        (pulse ? breatheVer : base).applyTo(view);
    }

    public void setAllianceColor() {
        switch (m_robotState.getAlliance()) {
            case RED     -> patternApplyTo(patternRed,    patternRedBreathe,    addressableLEDBuffer);
            case BLUE    -> patternApplyTo(patternBlue,   patternBlueBreathe,   addressableLEDBuffer);
            case MISSING -> patternApplyTo(patternPurple, patternPurpleBreathe, addressableLEDBuffer);
        }
    }

    public void setBatteryPulse() {
        pulse = m_robotState.getBatteryState() != BatteryMonitor.BatteryState.GOOD;
    }

    public void setRainbow(boolean forward) {
        LEDPattern base    = forward ? rainbowForward        : rainbowReversed;
        LEDPattern breathe = forward ? rainbowForwardBreathe : rainbowReversedBreathe;
        patternApplyTo(base, breathe, m_bottom);
        patternApplyTo(base, breathe, m_top);
    }

    public void setSolid(Color color) {
        LEDPattern.solid(color).applyTo(addressableLEDBuffer);
    }

    public void setViews() {
        // Right view is reversed because LED strips are wired in series
        m_bottom = addressableLEDBuffer.createView(Constants.Lights.leftViewStart, Constants.Lights.leftViewEnd);
        m_top    = addressableLEDBuffer.createView(Constants.Lights.rightViewStart, Constants.Lights.rightViewEnd).reversed();
    }

    private void switchLightState(HopperState state) {
        switch (state) {
            case IDLE, UNKNOWN -> setAllianceColor();
            case HOPPER_EMPTY  -> patternApplyTo(patternRed,    patternRedBreathe,    addressableLEDBuffer);
            case HOPPER_HALF   -> { patternApplyTo(patternRed, patternRedBreathe, m_top); patternApplyTo(patternGreen, patternGreenBreathe, m_bottom); }
            case HOPPER_FULL   -> patternApplyTo(patternGreen,  patternGreenBreathe,  addressableLEDBuffer);
            case HOPPER_LOW    -> patternApplyTo(patternOrange, patternOrangeBreathe, addressableLEDBuffer);
        }
    }
}
