package frc.robot.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveExtHome extends Command {

    private ExtensionSubsystem extension;
    private double targetExtension;

    public MoveExtHome(ExtensionSubsystem extension, double pos) {
        addRequirements(RobotContainer.m_extension);
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
