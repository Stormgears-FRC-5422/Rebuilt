package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;

import static edu.wpi.first.units.Units.*;

public class Lights extends StormSubsystem {
    public enum lightStates{
        IDLE,
        HOPPER_EMPTY,
        HOPPER_HALF,
        HOPPER_FULL,
        ALIGNED_TO_TAG,
        SWITCHING_SHIFTS
    };


    private final Timer lightTimer = new Timer();
    private static final double lightTimerSeconds = 1.0;
    private lightStates lastState;

    lightStates state = lightStates.IDLE;
    // Alliance variables
    private final RobotState robotState;
    Color RED_COLOR = Color.kRed;
    Color BLUE_COLOR = Color.kBlue;
    Color YELLOW_COLOR = Color.kYellow;
    Color GREEN_COLOR = Color.kGreen;
    Color ORANGE_COLOR = new Color(255, 32, 0);
    Color PINK_COLOR = new Color(225,105,180);

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private AddressableLEDBufferView m_bottom;
    private AddressableLEDBufferView m_top;

    private boolean pulse = false;
    private double pulsePeriod = 5.0;

    private int lEDLength = Constants.Lights.ledLength;
    private int halfLEDLength = Constants.Lights.ledLength/2;


    public Lights() {
        addressableLED = new AddressableLED(Constants.Lights.port);
        addressableLEDBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);
        m_bottom = addressableLEDBuffer.createView(0,halfLEDLength-1);
        m_top = addressableLEDBuffer.createView(halfLEDLength, lEDLength-1);


        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.start();
        setViews();
        state = lightStates.IDLE;
        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();
        LEDPattern red = LEDPattern.solid(Color.kRed);
        LEDPattern green = LEDPattern.solid(Color.kGreen);
        switch(state){
            case IDLE:
                setRainbow(pulse);
            case HOPPER_EMPTY:
                //red
                setSolid(RED_COLOR);
            case HOPPER_HALF:
                //Half green and half red
                red.applyTo(m_top);
                green.applyTo(m_bottom);
            case HOPPER_FULL:
                //All green
                setSolid(GREEN_COLOR);
            case ALIGNED_TO_TAG:
                setSolid(BLUE_COLOR);
                if (lightTimer.hasElapsed(lightTimerSeconds)) {
                    setLightState(lastState);
                }
            case SWITCHING_SHIFTS:
                setSolid(ORANGE_COLOR);
        }


        // Write the data to the LED strip
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    // Wrapper function to allow pulse effect to be applied to any pattern. Call this function rather
    // than calling pattern.applyTo()
    private <T extends LEDReader & LEDWriter> void patternApplyTo(LEDPattern basePattern, T view) {
        LEDPattern finalPattern = basePattern;

        if (pulse) {
            finalPattern = basePattern.breathe(Seconds.of(pulsePeriod));
        }

        finalPattern.applyTo(view);
    }

    public void setRainbow(boolean forward) {
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        Distance kLedSpacing = Meters.of(1 / 120.0);
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.45), kLedSpacing);
        if (!forward)
            scrollingRainbow = scrollingRainbow.reversed();

        patternApplyTo(scrollingRainbow, m_bottom);
        patternApplyTo(scrollingRainbow, m_top);
    }

    public void setSolid(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        patternApplyTo(pattern, addressableLEDBuffer);
    }

    public void setViews() {
        // Split led buffer into left and right views, so we can apply a different pattern to each strip
        // Right view is reversed because LED strips are wired in series
        m_bottom = addressableLEDBuffer.createView(Constants.Lights.leftViewStart, Constants.Lights.leftViewEnd);
        m_top = addressableLEDBuffer.createView(Constants.Lights.rightViewStart, Constants.Lights.rightViewEnd).reversed();
    }
    public void setLightState(lightStates newState){
        lastState = state;
        state = newState;
        if(newState == lightStates.ALIGNED_TO_TAG){
            lightTimer.start();
        }
    }
    public lightStates getLightState(){
        return state;
    }
}
