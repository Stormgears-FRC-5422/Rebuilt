package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;
import frc.robot.sensors.stormnet.StormNet;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class SensorSubsystem extends StormSubsystem {
    private final DigitalInput bottomBeamBreak;

    private StormNet stormNet;
    private RobotState robotState;

    private boolean bottomBB;
    private boolean stormNetReady;
    private double lidarDistance;
    private HopperState hopperState = HopperState.UNKNOWN;

    public SensorSubsystem() {
        bottomBeamBreak = new DigitalInput(Constants.SensorSubsystem.bottomBeamBreakPort);

        if (Constants.SensorSubsystem.useStormNet) {
            StormNet.init();
            stormNet = StormNet.getInstance();

            if (Constants.Debug.debug) {
                stormNet.test();
            }
        }

        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        updateHopperDepth();
        robotState.setHopperState(hopperState);

        Logger.recordOutput("SensorSubsystem/BottomBeamBreak", bottomBB);
        Logger.recordOutput("SensorSubsystem/StormNetReady", stormNetReady);
        Logger.recordOutput("SensorSubsystem/LidarDistance", lidarDistance);
        Logger.recordOutput("SensorSubsystem/HopperState", hopperState.toString());
    }

    // Compute hopper depth based on all available sensors.
    // Combines lidar (continuous distance) with beam break (binary).
    // Takes the "fuller" of the two readings as the conservative estimate.
    // When StormNet is disabled or not ready, falls back to beam break only.
    private void updateHopperDepth() {
        bottomBB = !bottomBeamBreak.get();
        stormNetReady = stormNet != null && stormNet.isReady();
        lidarDistance = stormNetReady ? stormNet.getLidarDistance() : 0.0;

        // Beam break: can only distinguish EMPTY from LOW
        HopperState bbState = bottomBB ? HopperState.HOPPER_LOW : HopperState.HOPPER_EMPTY;

        // Lidar: lower distance = fuller hopper. Negative means bad data.
        HopperState lidarState = HopperState.UNKNOWN;
        if (stormNetReady && lidarDistance >= 0) {
            if (lidarDistance < Constants.SensorSubsystem.lidarDepthFull) {
                lidarState = HopperState.HOPPER_FULL;
            } else if (lidarDistance < Constants.SensorSubsystem.lidarDepthHalf) {
                lidarState = HopperState.HOPPER_HALF;
            } else if (lidarDistance < Constants.SensorSubsystem.lidarDepthLow) {
                lidarState = HopperState.HOPPER_LOW;
            } else {
                lidarState = HopperState.HOPPER_EMPTY;
            }
        }

        // Combine: if lidar data is bad or unavailable, trust beam break alone.
        // Otherwise take whichever says more full (conservative).
        // Beam break can only report EMPTY or LOW, so the only interesting case
        // is lidar=EMPTY but beam break=LOW.
        if (lidarState == HopperState.UNKNOWN) {
            hopperState = bbState;
        } else if (lidarState == HopperState.HOPPER_EMPTY && bbState == HopperState.HOPPER_LOW) {
            hopperState = bbState;
        } else {
            hopperState = lidarState;
        }
    }

}
