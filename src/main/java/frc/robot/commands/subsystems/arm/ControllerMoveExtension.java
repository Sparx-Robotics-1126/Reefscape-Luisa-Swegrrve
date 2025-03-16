package frc.robot.commands.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;
import java.util.function.DoubleSupplier;

public class ControllerMoveExtension extends Command {

    private final DoubleSupplier m_power;
    private final ExtensionSubsystem m_arm;

    public ControllerMoveExtension(DoubleSupplier power, ExtensionSubsystem arm) {
        addRequirements(arm);
        m_power = power;
        m_arm = arm;
    }

    @Override
    public void execute() {
        double speed = MathUtil.applyDeadband(m_power.getAsDouble(), .1);
        m_arm.moveExtension(speed);
    }
}
