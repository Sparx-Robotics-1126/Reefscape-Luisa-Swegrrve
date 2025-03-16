package org.team1126.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
// import org.team1126.robot.RobotContainer;
import org.team1126.robot.subsystems.ExtensionSubsystem;

public class MoveExtHome extends Command {

   private ExtensionSubsystem extension;
   private double targetExtension;

   public MoveExtHome(ExtensionSubsystem extension, double pos) {
    //    addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       targetExtension = pos;
   }

   @Override
   public void execute() {
        extension.extReachGoal(targetExtension);
       
    
   }

   @Override
   public boolean isFinished() {
       return false;
   }

}
