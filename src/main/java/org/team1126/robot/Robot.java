package org.team1126.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team1126.lib.util.DisableWatchdog;
import org.team1126.lib.util.Profiler;
import org.team1126.lib.util.Tunable;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.ReefSelection;

@Logged
public final class Robot extends TimedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    
    public final Swerve swerve;

    // public final ReefSelection selection;


    private final CommandXboxController driver;
    // private final CommandXboxController coDriver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // DisableWatchdog.in(scheduler, "m_watchdog");
        // DisableWatchdog.in(this, "m_watchdog");

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
       
        swerve = new Swerve();

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        
        
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Driver bindings
       
        driver.x().onTrue(none()); // Reserved (No goosing around)
        driver.y().onTrue(none()); // Reserved (Force goose spit)

        // Set thread priority
        Threads.setCurrentThreadPriority(true, 10);
    }

    /**
     * Returns the current match time in seconds.
     */
    // public double matchTime() {
    //     return Math.max(DriverStation.getMatchTime(), 0.0);
    // }

    /**
     * Returns {@code true} if it is safe for the goose neck and elevator to move.
     */
   
    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getRightX();
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("robotPeriodic");
        Profiler.run("scheduler", scheduler::run);
        // Profiler.run("lights", lights::update);
        Profiler.run("epilogue", () -> Epilogue.update(this));
        Profiler.run("tunables", Tunable::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
