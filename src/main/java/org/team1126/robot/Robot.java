package org.team1126.robot;

// import static edu.wpi.first.wpilibj.XboxController.Axis.*;
// import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1126.lib.util.Profiler;
import org.team1126.lib.util.Tunable;
import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.commands.LED.RainbowCommand;
import org.team1126.robot.commands.LED.TeamLights;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ClimbSubsystem;
import org.team1126.robot.subsystems.ClimbSubsystem.ClimberPosition;
import org.team1126.robot.subsystems.ExtensionSubsystem;
import org.team1126.robot.subsystems.LEDs;
import org.team1126.robot.subsystems.PlacerSubsystem;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.subsystems.ArmSubsystem.ArmPosition;
import org.team1126.robot.subsystems.ExtensionSubsystem.ExtensionPosition;
import org.team1126.robot.util.ReefSelection;

@Logged
public final class Robot extends TimedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    
    public final Swerve swerve;
    public final ClimbSubsystem climber;
    public final ExtensionSubsystem extension;
    public final ArmSubsystem arm;
    public final PlacerSubsystem placer;
    public final Autos autos;
    public final LEDs leds;
    public final ReefSelection selection;
    public final Routines routines;


    private final CommandXboxController driver;
    private final CommandXboxController operator;
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
        climber = new ClimbSubsystem();
        extension = new ExtensionSubsystem(this);
        arm = new ArmSubsystem(this);
        placer = new PlacerSubsystem();
        swerve = new Swerve();
        leds = new LEDs(0, 300); // PORT IS PWM!!

        selection = new ReefSelection();

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        operator = new CommandXboxController(Constants.kOperator);
        
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));
        leds.setDefaultCommand(leds.setReefLights(false, 1126));
        // arm.setDefaultCommand(new ControllerMoveArm(()-> operator.getRawAxis(XboxController.Axis.kLeftY.value), arm));
       extension.setDefaultCommand(extension.goTo(ExtensionPosition.kHome, this::safeForExtension));

       routines = new Routines(this);
        autos = new Autos(this);
        

        //driver.leftTrigger().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        //driver.a().onTrue(new InstantCommand(() -> swerve.resetPose(null)));
        driver.y().whileTrue(climber.goTo(ClimberPosition.kHome));
        driver.x().whileTrue(climber.goTo(ClimberPosition.kIn));
        driver.b().whileTrue(climber.goTo(ClimberPosition.kOut));
        // driver.y().whileTrue(leds.setc)

        driver.leftStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));
        driver.leftBumper().onTrue(selection.setLeft()).whileTrue(swerve.driveReef(this::driverX, this::driverY, this::driverAngular,selection::isLeft));
        driver.rightBumper().onTrue(selection.setRight()).whileTrue(swerve.driveReef(this::driverX, this::driverY, this::driverAngular,selection::isLeft));
        driver.leftTrigger().onTrue(swerve.tareRotation());

        // Operator bindings
        operator.povDown().whileTrue(arm.goTo(ArmPosition.kHome)
            .alongWith(extension.goTo(ExtensionPosition.kHome,  ()-> true) )); //arm home

        operator.povUp().whileTrue(arm.goTo(ArmPosition.kCoralStation)
            .alongWith(extension.goTo(ExtensionPosition.kCoralStation, this::safeForExtension))   //arm to coral station
            .alongWith(placer.ingestCoral()).andThen(placer.positionCoral()));                                                  

        operator.a().whileTrue(arm.goTo(ArmPosition.kLevel1)
            .alongWith(extension.goTo(ExtensionPosition.kLevel1, this::safeForExtension))
            .alongWith(leds.setReefLights(true, 1))); //arm l1

        operator.x().whileTrue(arm.goTo(ArmPosition.kLevel2)
            .alongWith(extension.goTo(ExtensionPosition.kLevel2, this::safeForExtension))
            .alongWith(leds.setReefLights(true, 2))); //arm l2

        operator.b().whileTrue(arm.goTo(ArmPosition.kLevel3)
            .alongWith(extension.goTo(ExtensionPosition.kLevel3, this::safeForExtension))
            .alongWith(leds.setReefLights(true, 3))); //arm l3

        operator.y().whileTrue(arm.goTo(ArmPosition.kLevel4)
            .alongWith(extension.goTo(ExtensionPosition.kLevel4, this::safeForExtension))
            .alongWith(leds.setReefLights(true, 4))); //arm l4

        operator.rightTrigger(0.1).whileTrue(placer.analogPlacer(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value),false));
        operator.leftTrigger(0.1).whileTrue(placer.analogPlacer(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value),true));

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(autos.runSelectedAuto());
        RobotModeTriggers.autonomous().whileTrue(new RainbowCommand(leds));

        
        // driver.x().onTrue(none()); // Reserved (No goosing around)
        // driver.y().onTrue(none()); // Reserved (Force goose spit)

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
        return -driver.getRightX();
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
    public void disabledPeriodic() {
        var lights = new TeamLights(leds);
        lights.initialize();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void teleopInit() {
          CommandScheduler.getInstance().cancelAll();
        
    }

    public boolean safeForCoralStation() {
        return !placer.bottomHasCoral() && swerve.wildlifeConservationProgram();
    }

    public  double getArmPosition() {
        return arm.getArmAngle();
    }

    public boolean safeForExtension() {
        //System.out.println(arm.getArmAngle() > 7.5);
        return arm.getArmAngle() > 7.5;

    }
}
