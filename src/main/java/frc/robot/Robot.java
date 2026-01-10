// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState.StatePeriod;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    private RobotContainer robotContainer;
    private RobotState state;
    private int iteration = 0;

    public Robot() {
        state = RobotState.getInstance();

        switch (state.getSimMode()) {
            case REAL:
                console("This is a REAL robot");
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
//                LoggedPowerDistribution.getInstance(Constants.Power.moduleID, Constants.Power.isRevPdh ? PowerDistribution.ModuleType.kRev : PowerDistribution.ModuleType.kCTRE);
                break;
            case AKIT_REPLAY:
                console("This is an ADVANTAGE KIT REPLAY robot");
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                break;
            case AKIT_SIM:
                console("This is an ADVANTAGE KIT SIMULATION robot");
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                console("This is a SIMULATION robot");
                break;
        }

        if (Constants.Toggles.useAdvantageKit) {
            Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
            logBuildDetails();
        }

        try {
            robotContainer = new RobotContainer();
        } catch (Exception e) {
            robotContainer = null;
            console("can't create RobotContainer. Eating the following exception:");
            //noinspection CallToPrintStackTrace
            e.printStackTrace();
        }
    }

    @Override
    public void startCompetition() {
        boolean hold = false;
        Exception tmpException = null;

        try {
            super.startCompetition();
        } catch (Exception e) {
            if (Constants.Debug.debug && Constants.Debug.holdCrash) {
                tmpException = e;
                hold = true;
            } else {
                throw e; // Normal behavior to crash and restart program
            }
        }

        if (hold) {
            console("Holding console after crash due to unhandled exception");
            console("You will need to RESTART ROBOT CODE, REDEPLOY, or REBOOT ROBORIO to proceed");
            console("Message: " + tmpException.getMessage());
            console("Stack trace:");
            tmpException.printStackTrace();
            CommandScheduler.getInstance().cancelAll();
            try {
                while (true) {
                    Thread.sleep(1000);
                }
            } catch (Exception x) {
                x.printStackTrace();
            }
        }
    }

    private void logBuildDetails() {
        Logger.recordMetadata("ProjectName", Constants.robotName); // Set a metadata value
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 ->
                "All changes committed";
            case 1 ->
                "Uncommitted changes";
            default ->
                "Unknown";
        });
    }

    @Override
    public void robotInit() {
        if (Constants.Toggles.useWebServer) {
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (robotContainer != null) {
            robotContainer.periodic();
        }
        Options.periodic();
    }

    @Override
    public void disabledInit() {
        console("DisabledInit");
        state.setPeriod(StatePeriod.DISABLED);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        console("DisabledExit");
    }

    @Override
    public void autonomousInit() {
        console("AutoInit");
        state.setPeriod(StatePeriod.AUTONOMOUS);
    }

    @Override
    public void autonomousExit() {
        console("AutoExit");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        console("TeleopInit");
        state.setPeriod(StatePeriod.TELEOP);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        console("TeleopExit");
    }

    @Override
    public void testInit() {
        console("TestInit");
        state.setPeriod(StatePeriod.TEST);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        console("TestExit");
    }

    @Override
    public void simulationInit() {
        console("SimulationInit");
    }

    @Override
    public void simulationPeriodic() {
    }

    public void console(String message) {
        System.out.println("Robot : " + message);
    }

    public void console(String message, int iterations) {
        if (iteration % iterations == 0) {
            console(message);
        }
    }
}
