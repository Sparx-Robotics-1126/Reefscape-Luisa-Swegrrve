package org.team1126.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.ClimbSubsystem;

public class ClimbMoveArm extends Command {
    
    private final DoubleSupplier m_power;
    private final ClimbSubsystem m_climb;

    public ClimbMoveArm(DoubleSupplier power, ClimbSubsystem climb) {
        addRequirements(climb);
        m_power = power;
        m_climb = climb;
    }

    @Override
    public void execute() {
        double speed =  MathUtil.applyDeadband(m_power.getAsDouble(), .1);
        m_climb.moveClimb(speed);
    }
}
