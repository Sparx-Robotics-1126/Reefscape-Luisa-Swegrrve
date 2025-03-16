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
        // if(climb.getAngle() > targetAngle) {
            climb.climbReachGoal(targetAngle);
        // } else if(climb.getAngle() < targetAngle) {
        //     climb.moveClimbToPos(targetAngle);
        // }
    }

    @Override
    public boolean isFinished() {
        // if(climb.getAngle() < -160 || climb.getAngle() > 90) {
        //     return true;
        // }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        climb.moveClimb(0);
    }
}
