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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team1126.lib.util.DisableWatchdog;
import org.team1126.lib.util.Profiler;
import org.team1126.lib.util.Tunable;
import org.team1126.robot.Constants.AprilTagPositions;
import org.team1126.robot.Constants.ArmConstants;
import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.LED.RainbowCommand;
import org.team1126.robot.commands.LED.ReefLights;
import org.team1126.robot.commands.LED.TeamLights;
import org.team1126.robot.commands.arm.MoveArmToAngle;
import org.team1126.robot.commands.arm.MoveExtensionToPos;
import org.team1126.robot.commands.climb.ClimbMoveToPos;
import org.team1126.robot.commands.placer.AnalogPlacer;
import org.team1126.robot.commands.placer.IngestCoral;
import org.team1126.robot.commands.placer.PositionCoral;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ClimbSubsystem;
import org.team1126.robot.subsystems.ExtensionSubsystem;
import org.team1126.robot.subsystems.LEDs;
import org.team1126.robot.subsystems.PlacerSubsystem;
import org.team1126.robot.subsystems.Swerve;
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
    // public final ReefSelection selection;


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
        extension = new ExtensionSubsystem();
        arm = new ArmSubsystem();
        placer = new PlacerSubsystem();
        swerve = new Swerve();
        leds = new LEDs(0, 300); // PORT IS PWM!!

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        operator = new CommandXboxController(Constants.kOperator);
        
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));
        leds.setDefaultCommand(new TeamLights(leds));

        autos = new Autos(this);

        //driver.leftTrigger().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        //driver.a().onTrue(new InstantCommand(() -> swerve.resetPose(null)));
        driver.y().whileTrue(new ClimbMoveToPos(climber, 0));
        driver.x().whileTrue(new ClimbMoveToPos(climber, 125));
        driver.b().whileTrue(new ClimbMoveToPos(climber, -112.55));
        // driver.leftBumper().whileTrue(new DriveToClosestLeftBranchPoseCommand(swerve));  
        // driver.leftBumper().whileTrue(swerve.driveToPose(swerve.getClosestLeftBranchPose()));
        // driver.leftBumper().whileTrue(swerve.driveToPose(AprilTagPositions.APRILTAGS_BLU[0]));
        // driver.rightBumper().whileTrue(swerve.driveToPose(swerve.getClosestRightBranchPose()));

        // Operator bindings
        operator.povDown().whileTrue(new MoveArmToAngle(arm, 0).alongWith(new MoveExtensionToPos(extension, arm, 0.01))); //arm home
        operator.povUp().whileTrue(new MoveArmToAngle(arm, 18.442849922180176).alongWith(new MoveExtensionToPos(extension, arm, .01))
                  .alongWith(new IngestCoral(placer, -.5)).andThen(new PositionCoral(placer)));                                                    //arm to coral station

        operator.a().whileTrue(new MoveArmToAngle(arm, ArmConstants.L1_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, 0.013659)).alongWith(new ReefLights(leds, true, 1))); //arm l1
        operator.x().whileTrue(new MoveArmToAngle(arm, ArmConstants.L2_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm,-0.0831989)).alongWith(new ReefLights(leds, true, 2))); //arm l2
        operator.b().whileTrue(new MoveArmToAngle(arm,  ArmConstants.L3_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, -0.25)).alongWith(new ReefLights(leds, true, 3))); //arm l3
        operator.y().whileTrue(new MoveArmToAngle(arm, ArmConstants.L4_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, -0.55)).alongWith(new ReefLights(leds, true, 4))); //arm l4

        operator.rightTrigger(0.1).whileTrue(new AnalogPlacer(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value), placer,false));
        operator.leftTrigger(0.1).whileTrue(new AnalogPlacer(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value), placer,true));

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
