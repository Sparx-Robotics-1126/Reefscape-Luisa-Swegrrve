package org.team1126.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.ExtensionSubsystem;


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
        double speed =  MathUtil.applyDeadband(m_power.getAsDouble(), .1);
        m_arm.moveExtension(speed);
    }
    
}
