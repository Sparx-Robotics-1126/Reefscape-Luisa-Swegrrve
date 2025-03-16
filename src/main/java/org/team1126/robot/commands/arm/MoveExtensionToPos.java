package org.team1126.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
// import org.team1126.robot.RobotContainer;
import org.team1126.robot.subsystems.ArmSubsystem;
import org.team1126.robot.subsystems.ExtensionSubsystem;


public class MoveExtensionToPos extends Command {

   private ExtensionSubsystem extension;
   private ArmSubsystem arm;
   private double targetExtension;

   public MoveExtensionToPos(ExtensionSubsystem extension, ArmSubsystem arm, double pos) {
    //    addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       this.arm = arm;
       targetExtension = pos;
   }

   @Override
   public void execute() {
    if(arm.getArmAngle() > 7.5 ){
    if(Math.abs(targetExtension) > 0) {
        extension.extReachGoal(targetExtension);
    }
}
       
    
   }

   @Override
   public boolean isFinished() {
       return false;
   }

}
