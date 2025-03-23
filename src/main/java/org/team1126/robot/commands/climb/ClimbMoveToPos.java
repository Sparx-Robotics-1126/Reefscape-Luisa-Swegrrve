package org.team1126.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.ClimbSubsystem;


public class ClimbMoveToPos extends Command {
    
    private ClimbSubsystem climb;
    private double targetAngle;

    public ClimbMoveToPos(ClimbSubsystem climbSubsystem, double angle) {
       //addRequirements(RobotContainer.m_climb);
        climb = climbSubsystem;
        targetAngle = angle;
    }

    @Override
    public void execute() {

            climb.climbReachGoal(targetAngle);
            climb.setBeachMode(false);

    }

    @Override
    public boolean isFinished() {
        climb.setBeachMode(true);
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        climb.moveClimb(0);
    }
}
