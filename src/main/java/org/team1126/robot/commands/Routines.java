package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.team1126.robot.Robot;
import org.team1126.robot.Constants.ArmConstants;
import org.team1126.robot.Constants.ExtensionConstants;
import org.team1126.robot.commands.arm.MoveArmToAngle;
import org.team1126.robot.commands.arm.MoveExtHome;
import org.team1126.robot.commands.arm.MoveExtensionToPos;
// import org.team1126.robot.commands.placer.AcquireCoral;
// import org.team1126.robot.commands.placer.IngestCoral;
// import org.team1126.robot.commands.placer.PlaceCoral;
// import org.team1126.robot.commands.placer.PositionCoral;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ExtensionSubsystem;
// import org.team1126.robot.subsystems.Climber;
// import org.team1126.robot.subsystems.ExtensionSubsystem;
// import org.team1126.robot.subsystems.Elevator.ElevatorPosition;
// import org.team1126.robot.subsystems.GooseNeck;
// import org.team1126.robot.subsystems.Intake;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.PlacerSubsystem;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.ReefSelection;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Robot robot;
    private final ArmSubsystem arm;
    private final ExtensionSubsystem extension;
    private final Swerve swerve;
    private final PlacerSubsystem placer;

    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        swerve = robot.swerve;
        selection = robot.selection;
        arm = robot.arm;
        extension = robot.extension;
        placer = robot.placer;
    }

    public Command driveToCoral(boolean isRight) {

        return parallel(
            swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft).withTimeout(4)
        );
    }

public Command safe(BooleanSupplier button){
    return deadline(  new MoveExtensionToPos(extension, arm,.01),
                    new MoveArmToAngle(arm, 18.442849922180176))
    .withName("Routines.safe()");
}

    public Command toL4(ArmSubsystem arm, ExtensionSubsystem extension) {

        return parallel(
            new MoveArmToAngle(arm, ArmConstants.L4_ARM_POS),
            new MoveExtensionToPos(extension, arm, ExtensionConstants.L4_EXT_POS)
        );
    }

    public Command placeL4(ArmSubsystem arm, ExtensionSubsystem extension, PlacerSubsystem placer) {

        return 
            parallel(
                new MoveArmToAngle(arm, ArmConstants.L4_ARM_POS),
                new MoveExtensionToPos(extension, arm, ExtensionConstants.L4_EXT_POS),
                sequence (
                    waitUntil(swerve::inRange),
                    placer.placeCoral( .3).withTimeout(1)
                )
                
            );
    }

    public Command moveHome(ArmSubsystem arm, ExtensionSubsystem extension){

        return parallel(
            new MoveArmToAngle(arm, -.01),
            new MoveExtHome(extension, 0)
        );
    }

    public Command toCoral(ArmSubsystem arm, ExtensionSubsystem extension) {

        return parallel(
            new MoveArmToAngle(arm, 18.442849922180176),
            new MoveExtensionToPos(extension, arm, .01)
        );
    }
    public Command intakeCoral(PlacerSubsystem placer) {
        return sequence(
            placer.ingestCoral(),
            placer.positionCoral()
        );
    }


    
    /**
     * Scores a coral, with driver assists. The drivetrain will be "pushed" to
     * center itself on a reef pipe, and the robot will also face the reef.
     * @param runManual A boolean supplier that when {@code true} will force the goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command assistedScore(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        return parallel(
            // score(runManual, allowGoosing),
            swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft)
        ).withName("Routines.assistedScore()");
    }
    public Command assistedScore(BooleanSupplier runManual) {
        return parallel(
            // score(runManual, allowGoosing),
            swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft)
        ).withName("Routines.assistedScore()");
    }

}
