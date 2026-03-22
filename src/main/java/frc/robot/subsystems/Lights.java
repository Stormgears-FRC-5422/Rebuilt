
package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;
import frc.utils.StormSubsystem;
import frc.robot.subsystems.misc.BatteryMonitor;
import frc.robot.util.GameDataTracker;
import frc.robot.util.GameDataTracker.ShiftInfo;


import static edu.wpi.first.units.Units.*;

import java.lang.reflect.InaccessibleObjectException;
import java.util.Optional;

public class Lights extends StormSubsystem {

    private RobotState.HopperState hopperState;

    // Alliance variables
    private final RobotState m_robotState;


    Color RED_COLOR = Color.kRed;
    Color BLUE_COLOR = Color.kBlue;
    Color YELLOW_COLOR = Color.kYellow;
    Color GREEN_COLOR = Color.kGreen;
    Color ORANGE_COLOR = new Color(255, 32, 0);
    Color PINK_COLOR = new Color(225,105,180);
    Color PURPLE_COLOR = Color.kPurple;

    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private AddressableLEDBufferView m_bottom;
    private AddressableLEDBufferView m_top;

    private boolean pulse = false;
    private double pulsePeriod = 5.0;

    private int lEDLength = Constants.Lights.ledLength;
    private int halfLEDLength = Constants.Lights.ledLength/2;

    private boolean inActiveFirst;

    private enum hubStates {
        AUTO,
        ACTIVE_SHIFT,
        TRANSITION_PERIOD,
        INACTIVE_SHIFT,
        UNKNOWN,
    }

    hubStates curHubStates = hubStates.UNKNOWN;
    // private String alliance = DriverStation.getAlliance().toString();
    private Optional<DriverStation.Alliance> alliance;
    private String team;
    
public Lights() {
    addressableLED = new AddressableLED(Constants.Lights.port);
    addressableLEDBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);
    m_bottom = addressableLEDBuffer.createView(0,halfLEDLength-1);
    m_top = addressableLEDBuffer.createView(halfLEDLength, lEDLength-1);


    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.start();
    setViews();
    m_robotState = RobotState.getInstance();

    //get alliance from driver station
    alliance = DriverStation.getAlliance();

}

@Override
public void periodic() {
    super.periodic();
    
    hopperState = m_robotState.getHopperState();

    // if(checkLightState(hopperState)){ // If the hopper state has changed
    //     setLightState(hopperState);
    //     switchLightState();
    // }

    setBatteryPulse();

    switchShift();

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
public void setAllianceColor(){
    if(m_robotState.getAlliance() == RobotState.StateAlliance.RED){
        setSolid(RED_COLOR);
    }
    else if(m_robotState.getAlliance() == RobotState.StateAlliance.BLUE){
        setSolid(BLUE_COLOR);
    }
    else if(m_robotState.getAlliance() == RobotState.StateAlliance.MISSING){
        setSolid(PURPLE_COLOR);
    }
}

public void setBatteryPulse(){
    if(m_robotState.getBatteryState() == BatteryMonitor.BatteryState.GOOD){
        pulse = false;
    }
    else if(m_robotState.getBatteryState() == BatteryMonitor.BatteryState.WARN){
        pulse = true;
    }
    else if(m_robotState.getBatteryState() == BatteryMonitor.BatteryState.BAD){
        pulse = true;

    }
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

public void setLightState(HopperState newState){
    hopperState = newState;
}
public HopperState getLightState(){
    return hopperState;
}
public boolean checkLightState(HopperState newState){
    if(hopperState != newState){
        return true;
    }
    return false;
}
public void switchLightState(){

    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern green = LEDPattern.solid(Color.kGreen);

    switch(hopperState){
        case IDLE:
            setAllianceColor();
            break;
        case HOPPER_EMPTY:
            setSolid(RED_COLOR);
            break;
        case HOPPER_HALF:
            red.applyTo(m_top);
            green.applyTo(m_bottom);
            break;
        case HOPPER_FULL:
            setSolid(GREEN_COLOR);
            break;
    }
}

public void checkShift(){
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0)
    {
        switch (gameData.charAt(0)) {
            case 'B' :
                //Blue case code
                if(team.equals("BLUE")){
                    inActiveFirst = true;
                } 
                else inActiveFirst = false;
                break;
            case 'R' :
                //Red case code
                if(team.equals("RED")){
                    inActiveFirst = true;
                }
                else inActiveFirst = false;
                break;
            default :
                break;
        }
    } 
    else {
    //Code for no data received yet
    }
}
public boolean switchingShiftSoon(){
    for(int i = 4; i >=0; i--){
        int transitionTime = 130 - (i*25);
        if(DriverStation.getMatchTime() < transitionTime && DriverStation.getMatchTime() > transitionTime -7){
            return true;
        }
    }
    return false;
}
//AUTO - 20sec, 10(transition), 25, 25, 25, 25, 30(end game)

public void setShift(boolean inActiveFirst){

    int matchTime = (int) DriverStation.getMatchTime();
    if(inActiveFirst){
        if(matchTime<130 && matchTime > 105){
            setSolid(RED_COLOR);
        }
        else if(matchTime<105 && matchTime > 80){
            setSolid(GREEN_COLOR);
        }
        else if(matchTime<80 && matchTime > 55){
            setSolid(RED_COLOR);
        }
        else if(matchTime<55 && matchTime > 30){
            setSolid(GREEN_COLOR);
        }
        else if(matchTime<30 && matchTime > 0){
            setSolid(GREEN_COLOR);
        }
    }
    else{
        if(matchTime<130 && matchTime > 105){
            setSolid(GREEN_COLOR);
        }
        else if(matchTime<105 && matchTime > 80){
            setSolid(RED_COLOR);
        }
        else if(matchTime<80 && matchTime > 55){
            setSolid(GREEN_COLOR);
        }
        else if(matchTime<55 && matchTime > 30){
            setSolid(RED_COLOR);
        }
        else if(matchTime<30 && matchTime > 0){
            setSolid(GREEN_COLOR);
        }
    }
}
public void setTransitionPeriod(boolean transition){
        if(transition){
            setRainbow(pulse);
        }
    }

public void updateAlliance(){
    if (alliance.isPresent()) {
        if (alliance.get().equals(DriverStation.Alliance.Red)) {
            team = "RED";
        }
        if (alliance.get().equals(DriverStation.Alliance.Blue)) {
            team = "BLUE";
        }
    }
    else {
        team = "MISSING"; // Default value if alliance is not detected
    }
}
public void switchShift(){

    ShiftInfo info = GameDataTracker.getShiftInfo();

    switch(info.currentShift()){ 
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
            if(info.ourShift()){
                setSolid(GREEN_COLOR);
            }
            else{
                setSolid(RED_COLOR);
            }
            if (info.remainingTime() < 7.0) {
                setSolid(YELLOW_COLOR);
            }
            break; 
        }
    }
    
}

