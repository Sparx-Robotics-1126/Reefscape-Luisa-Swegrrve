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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1126.robot.Robot;
import org.team1126.robot.Constants.ArmConstants;
import org.team1126.robot.commands.arm.MoveArmToAngle;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ExtensionSubsystem;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.PlacerSubsystem;
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
    private final ArmSubsystem arm;
    private final ExtensionSubsystem extension;
    private final PlacerSubsystem placer;

    private final Routines routines;
    // private final ReefSelection selection;

    private final AutoFactory factory;
    private final AutoChooser chooser;

    public Autos(Robot robot) {
        this.robot = robot;

     
        swerve = robot.swerve;
        arm = robot.arm;
        extension = robot.extension;
        placer = robot.placer;

        // selection = robot.selection;
        routines = robot.routines;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.addRoutine("Move Test", () -> moveTestAutoRoutine(false));
        chooser.addRoutine("J start test", () -> JTest(false));
        chooser.addRoutine("Forward l4", () -> l4Straight(false));
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
    //System.out.println("Creating Move Test Auto Routine");
    AutoRoutine routine = factory.newRoutine("Right l4 (1)");

    AutoTrajectory part1 = routine.trajectory("Right l4 (1)", mirror);
    AutoTrajectory part2 = routine.trajectory("Right l4 (2)", mirror);

    routine
    .active()
    .onTrue(
        sequence(
            parallel(
                part1.resetOdometry(),
                swerve.resetAutoPID()
            ),
            part1.spawnCmd()
        )

    );
    
    part1.active().onTrue(routines.toL4(arm, extension));
    
    part1.atTime(.75).onTrue(routines.placeL4(arm, extension, placer));
    part1.chain(part2);
    part2.atTime(.75).onTrue(routines.toCoral(arm, extension).withTimeout(3));



    return routine;
}

private AutoRoutine l4Straight(boolean mirror) {
    AutoRoutine routine = factory.newRoutine("StraightPath");

    AutoTrajectory straightPath = routine.trajectory("StraightPath", mirror);

    routine
    .active()
    .onTrue(
        sequence(
            parallel(
                swerve.resetAutoPID()
            ),
            straightPath.spawnCmd()
        )
    );
    
    routines.toL4(arm, extension).withTimeout(1);
    straightPath.active().onTrue(routines.toL4(arm, extension).withTimeout(3).andThen(routines.placeL4(arm, extension, placer).withTimeout(1)));


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
