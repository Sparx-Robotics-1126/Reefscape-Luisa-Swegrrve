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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1126.lib.util.DisableWatchdog;
import org.team1126.lib.util.Profiler;
import org.team1126.lib.util.Tunable;
import org.team1126.robot.Constants.ArmConstants;
// import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.commands.LED.ReefLights;
import org.team1126.robot.commands.LED.TeamLights;
import org.team1126.robot.commands.arm.ControllerMoveArm;
import org.team1126.robot.commands.arm.MoveArmToAngle;
import org.team1126.robot.commands.arm.MoveExtHome;
import org.team1126.robot.commands.arm.MoveExtensionToPos;
import org.team1126.robot.commands.placer.AnalogPlacer;
import org.team1126.robot.commands.placer.IngestCoral;
import org.team1126.robot.commands.placer.PositionCoral;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ClimbSubsystem;
import org.team1126.robot.subsystems.ExtensionSubsystem;
import org.team1126.robot.subsystems.LEDs;
// import org.team1126.robot.subsystems.Climber;
// import org.team1126.robot.subsystems.Elevator;
// import org.team1126.robot.subsystems.Elevator.ElevatorPosition;
// import org.team1126.robot.subsystems.GooseNeck;
// import org.team1126.robot.subsystems.Intake;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.PlacerSubsystem;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.ReefSelection;

@Logged
public final class Robot extends TimedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final ClimbSubsystem climber;
    public final ExtensionSubsystem extension;
    public final ArmSubsystem arm;
    public final PlacerSubsystem placer;
    // public final Lights lights;
    public final Swerve swerve;
    public final LEDs leds;

    public final ReefSelection selection;

    public final Routines routines;
    // public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController operator;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DisableWatchdog.in(scheduler, "m_watchdog");
        DisableWatchdog.in(this, "m_watchdog");

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
        leds = new LEDs(0,200);
        // lights = new Lights(); //TODO: May need to change
        swerve = new Swerve();

        // Initialize helpers
        selection = new ReefSelection();

        // Initialize compositions
        routines = new Routines(this);
        // autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        operator = new CommandXboxController(Constants.kOperator);

        // Create triggers
        // RobotModeTriggers.autonomous().whileTrue(autos.runSelectedAuto());
        // Trigger gooseAround = driver.x().negate().and(operator.a().negate());

        // Setup lights
        // lights.disabled().until(this::isEnabled).schedule();
        //TODO: Revisit
        // RobotModeTriggers.disabled().whileTrue(lights.disabled());
        // RobotModeTriggers.autonomous().whileTrue(lights.sides.flames());
        // RobotModeTriggers.teleop().whileTrue(lights.sides.levelSelection(selection));

        //TODO:  Look into this
        // new Trigger(this::isEnabled)
        //     .and(gooseNeck::hasCoral)
        //     .onTrue(lights.top.hasCoral(gooseNeck::goosing, gooseNeck::getPosition, selection))
        //     .onFalse(lights.top.scored().onlyIf(this::isEnabled));

        // Set default commands
         leds.setDefaultCommand(new TeamLights(leds));
        extension.setDefaultCommand(new MoveExtHome(extension, .05));
        arm.setDefaultCommand(new ControllerMoveArm(()-> operator.getRawAxis(XboxController.Axis.kLeftY.value), arm));
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));
        
        configureDriverBindings();
        configureOperatorBindings();
        // Co-driver bindings
        // coDriver.a().onTrue(none()); // Reserved (No goosing around)
        // coDriver.y().whileTrue(routines.climb());

        // coDriver.leftStick().and(coDriver.rightStick()).toggleOnTrue(climber.coastMode());

        // coDriver.povUp().onTrue(selection.incrementLevel());
        // coDriver.povDown().onTrue(selection.decrementLevel());

        // Set thread priority
        Threads.setCurrentThreadPriority(true, 10);
    }

private void configureDriverBindings(){

        // Driver bindings
        // driver.a().onTrue(routines.intake(driver.a()));
        // driver.b().whileTrue(routines.swallow());
        // driver.x().onTrue(none()); // Reserved (No goosing around)
        // driver.y().onTrue(none()); // Reserved (Force goose spit)

        // driver.start().whileTrue(routines.babyBird(driver.start()));

        driver.leftStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));
        driver.axisLessThan(kRightY.value, -0.5).onTrue(selection.incrementLevel());
        driver.axisGreaterThan(kRightY.value, 0.5).onTrue(selection.decrementLevel());

        // driver.povUp().whileTrue(routines.barf());
        // driver.povDown().whileTrue(routines.swallow());
        driver.povLeft().onTrue(swerve.tareRotation());

        driver.leftBumper().onTrue(selection.setLeft()).whileTrue(routines.assistedScore(driver.y() ));
        driver.rightBumper().onTrue(selection.setRight()).whileTrue(routines.assistedScore(driver.y()));

}

      public void configureOperatorBindings() {   

        operator.povDown().whileTrue(new MoveArmToAngle(arm, 0).alongWith(new MoveExtensionToPos(extension, arm, 0.01))); //arm home
        operator.povUp().whileTrue(new MoveArmToAngle(arm, 18.442849922180176).alongWith(new MoveExtensionToPos(extension, arm, .01))
                  .alongWith(new IngestCoral(placer, -.5)).andThen(new PositionCoral(placer)));                                                    //arm to coral station

                  operator.a().whileTrue(new MoveArmToAngle(arm, ArmConstants.L1_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, 0.013659)).alongWith(new ReefLights(leds, true, 1))); //arm l1
                  operator.x().whileTrue(new MoveArmToAngle(arm, ArmConstants.L2_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm,-0.0831989)).alongWith(new ReefLights(leds, true, 2))); //arm l2
                  operator.b().whileTrue(new MoveArmToAngle(arm,  ArmConstants.L3_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, -0.25)).alongWith(new ReefLights(leds, true, 3))); //arm l3
                  operator.y().whileTrue(new MoveArmToAngle(arm, ArmConstants.L4_ARM_POS).alongWith(new MoveExtensionToPos(extension, arm, -0.55)).alongWith(new ReefLights(leds, true, 4))); //arm l4

                  operator.rightTrigger(0.1).whileTrue(new AnalogPlacer(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value), placer,false));
                  operator.leftTrigger(0.1).whileTrue(new AnalogPlacer(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value), placer,true));
        // m_operator.povRight().whileTrue(new AlgaeMoveToPosition(m_algae, 40));
        // m_operator.leftBumper().whileTrue(new AlgaeMoveToHome(m_algae));
        // m_operator.rightBumper().whileTrue(new SpitAlgae(m_algae));
    }


    /**
     * Returns the current match time in seconds.
     */
    public double matchTime() {
        return Math.max(DriverStation.getMatchTime(), 0.0);
    }

    /**
     * Returns {@code true} if it is safe for the goose neck and elevator to move.
     */
    public boolean safeForGoose() {
        return false;
        // return !gooseNeck.beamBroken() && swerve.wildlifeConservationProgram();
    }

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
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("robotPeriodic");
        Profiler.run("scheduler", scheduler::run);
        // Profiler.run("lights", lights::update);//TODO: May need to change
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
