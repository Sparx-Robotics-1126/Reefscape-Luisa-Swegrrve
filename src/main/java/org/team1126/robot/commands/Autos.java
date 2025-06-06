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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

    private final AutoFactory factory;
    private final AutoChooser chooser;

    public Autos(Robot robot) {
        this.robot = robot;

     
        swerve = robot.swerve;
        arm = robot.arm;
        extension = robot.extension;
        placer = robot.placer;
        routines = robot.routines;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.addRoutine("2 Note RIGHT", () -> SideCoralAuto(false));
        chooser.addRoutine("2 Note LEFT", () -> SideCoralAuto(true));
        chooser.addRoutine("Straight L4", () -> L4Test(false));
        // chooser.addRoutine("TEST AUTO", () -> TestLine(false));
        // chooser.addRoutine("FULL MOVE", () -> wholeMove(false));
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */

    public Command runSelectedAuto() {

        System.out.println("Running auto: " + chooser.selectedCommandScheduler().getName());
        return chooser.selectedCommandScheduler();
    }

    private AutoRoutine SideCoralAuto(boolean mirror){
        //System.out.println("Creating Move Test Auto Routine");
        AutoRoutine routine = factory.newRoutine("Right L4 (1)");

        AutoTrajectory part1 = routine.trajectory("Right L4 (1)", mirror);
        //might need a small backup command to move away from reef
        AutoTrajectory part2 = routine.trajectory("Right L4 (2)", mirror);
        AutoTrajectory part3 = routine.trajectory("Right L4 (3)", mirror);
        AutoTrajectory part4 = routine.trajectory("Right L4 (4)", mirror);

        routine
        .active()
        .onTrue(
            sequence(
                parallel(
                    part1.resetOdometry(),
                    swerve.resetAutoPID()
                    
                ),
                part1.spawnCmd(),
                Commands.print("move to right L started")
            )

        );
        
        part1.atTime(.6).onTrue(routines.toCoral(arm, extension));
        part1.atTime(.75).onTrue(routines.toL4(arm, extension));
        part1.done()
        .onTrue(
            Commands.sequence(
                parallel(
        
                        routines.driveToCoral(true),
                        // routines.driveToCoral(true)
                        routines.toL4(arm, extension)
                    ).withTimeout(1),
                    
                    Commands.sequence(routines.placeL4(arm, extension, placer)).withTimeout(.75),
                    Commands.parallel(part2.spawnCmd()) //might need to work on timing for arm
        ));

        part2.active().onTrue(sequence(Commands.waitSeconds(1),routines.toCoral(arm, extension).withTimeout(2)));
        part2.done()
            .onTrue(Commands.sequence(
                new InstantCommand(() -> swerve.stop(false)),
                Commands.parallel(
                    routines.toCoral(arm, extension),
                    Commands.sequence(
                        routines.intakeCoral(placer),
                        // new WaitUntilCommand( () -> placer.coralClear()),
                        part3.spawnCmd()
                        
                    )

                )
                )
                
            );

        part3.atTime(.5).onTrue(routines.toL4(arm, extension).alongWith(routines.positionCoral(placer)));
        part3.done()
        .onTrue(
            Commands.sequence(
                parallel(
                    
                        routines.driveToCoral(true),
                        routines.toL4(arm, extension)
                    ).withTimeout(1.5),
                    
                    Commands.sequence(routines.placeL4(arm, extension, placer)).withTimeout(.75),
                    Commands.parallel(part4.spawnCmd())
            )
        );

        part4.atTime(.5).onTrue(routines.toCoral(arm, extension));

        return routine;
    }

    private AutoRoutine L4Test(boolean mirror) {

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

        straightPath.atTime(.05).onTrue(routines.toCoral(arm, extension).withTimeout(.25));
        straightPath.atTime(.5).onTrue(routines.toL4(arm, extension));

        straightPath.done().onTrue(
            sequence(
                parallel(
                    routines.toL4(arm, extension),
                    routines.driveToCoral(true)
                ).withTimeout(1),
                parallel(
                    routines.placeL4(arm, extension, placer)

                ).withTimeout(2),
                routines.toCoral(arm, extension).withTimeout(3)
                
            )

        );

        return routine;

    }
    
    //                                                                                     ***** TEST AUTO ROUTINES ***** 

    private AutoRoutine moveTestAutoRoutine(boolean mirror){
        //System.out.println("Creating Move Test Auto Routine");
        AutoRoutine routine = factory.newRoutine("Right L4 (1)");

        AutoTrajectory part1 = routine.trajectory("Right L4 (1)", mirror);
        AutoTrajectory part2 = routine.trajectory("Right L4 (2)", mirror);

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

        routine.observe(placer::bottomHasCoral).onTrue(routines.placeL4(arm, extension, placer).withTimeout(2));
        part1.active().onTrue(routines.toL4(arm, extension).withTimeout(4));
        part1.atTime(1.2).onTrue(routines.driveToCoral(false).withTimeout(2));
        System.out.println("going to part 2");
        part1.chain(part2);
        System.out.println("after");
        part2.active().onTrue(routines.toCoral(arm, extension).withTimeout(3));

        return routine;
    }

    private AutoRoutine wholeMove(boolean mirror){
        //System.out.println("Creating Move Test Auto Routine");
        AutoRoutine routine = factory.newRoutine("Right L4 (1)");

        AutoTrajectory part1 = routine.trajectory("Right L4 (1)", mirror);
        //might need a small backup command to move away from reef
        AutoTrajectory part2 = routine.trajectory("Right L4 (2)", mirror);
        AutoTrajectory part3 = routine.trajectory("Right L4 (3)", mirror);
        AutoTrajectory part4 = routine.trajectory("Right L4 (4)", mirror);

        routine
        .active()
        .onTrue(
            sequence(
                parallel(
                    part1.resetOdometry(),
                    swerve.resetAutoPID()
                    
                ),
                part1.spawnCmd(),
                Commands.print("move to right L started")
            )

        );

        part1.done()
        .onTrue(
                    Commands.parallel(part2.spawnCmd()) //might need to work on timing for arm
        );

        part2.active().onTrue(sequence(Commands.waitSeconds(1),routines.toCoral(arm, extension).withTimeout(2)));
        part2.done()
            .onTrue(Commands.sequence(
                        part3.spawnCmd()

                )
            );

        part3.done()
        .onTrue(

                    Commands.parallel(part4.spawnCmd())
        );

        part4.atTime(.5).onTrue(routines.toCoral(arm, extension));

        return routine;
    }

    public AutoRoutine TestLine(boolean mirror) {
    AutoRoutine routine = factory.newRoutine("TestPath");

    AutoTrajectory straightPath = routine.trajectory("Right L4 (1)", mirror);

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
    
    straightPath.done().onTrue(routines.driveToCoral(mirror).withTimeout(2));

    return routine;
} 
}
