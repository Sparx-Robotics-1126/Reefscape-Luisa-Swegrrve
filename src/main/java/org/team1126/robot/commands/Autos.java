package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1126.robot.Robot;

import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.ReefSelection;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private final Robot robot;

    private final Swerve swerve;

    // private final Routines routines;
    // private final ReefSelection selection;

    private final AutoFactory factory;
    private final AutoChooser chooser;

    public Autos(Robot robot) {
        this.robot = robot;

     
        swerve = robot.swerve;

        // selection = robot.selection;
        // routines = robot.routines;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.addRoutine("Move Test", () -> moveTestAutoRoutine(false));
        chooser.addRoutine("J start test", () -> JTest(false));
        // chooser.addRoutine("Right L4 x3 (Hopper)", () -> l4x3Hopper(true));
        // chooser.addRoutine("Left L4 x3 (Baby Bird)", () -> l4x3BabyBird(false));
        // chooser.addRoutine("Right L4 x3 (Baby Bird)", () -> l4x3BabyBird(true));
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public Command runSelectedAuto() {

        System.out.println("Running auto: " + chooser.selectedCommandScheduler().getName());
        return chooser.selectedCommandScheduler();
    }

private AutoRoutine moveTestAutoRoutine(boolean mirror){
    System.out.println("Creating Move Test Auto Routine");
    AutoRoutine routine = factory.newRoutine("Move Test");

    AutoTrajectory testPath = routine.trajectory("TestPath", mirror);

    routine
    .active()
    .onTrue(
        sequence(
            parallel(
                testPath.resetOdometry(),
                swerve.resetAutoPID()
            ),
            testPath.spawnCmd()
        )
    );

    return routine;
}

private AutoRoutine JTest(boolean mirror){
    AutoRoutine routine = factory.newRoutine("Start-J");

    AutoTrajectory testPath = routine.trajectory("Start-J", mirror);

    routine
    .active()
    .onTrue(
        sequence(
            parallel(
                swerve.resetAutoPID()
            ),
            testPath.spawnCmd()
        )
    );

    return routine;
}

     
}
