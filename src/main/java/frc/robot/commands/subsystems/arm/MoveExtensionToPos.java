package frc.robot.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveExtensionToPos extends Command {

    private ExtensionSubsystem extension;
    private ArmSubsystem arm;
    private double targetExtension;

    public MoveExtensionToPos(ExtensionSubsystem extension, ArmSubsystem arm, double pos) {
        addRequirements(RobotContainer.m_extension);
        this.extension = extension;
        this.arm = arm;
        targetExtension = pos;
    }

    @Override
    public void execute() {
        if (arm.getArmAngle() > 7.5) {
            if (Math.abs(targetExtension) > 0) {
                extension.extReachGoal(targetExtension);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
