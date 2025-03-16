package org.team1126.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.ClimbSubsystem;

public class ClimbMoveUntil extends Command{
    
public ClimbSubsystem climb;

public double targetAngle;

    public ClimbMoveUntil(ClimbSubsystem climb, double angle){
        //addRequirements(RobotContainer.m_climb);
        this.climb = climb;
        targetAngle = angle;
    }


    @Override
    public void execute(){
        if(climb.getAngle() < targetAngle){
            climb.moveClimb(2);
        }
    }


    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
