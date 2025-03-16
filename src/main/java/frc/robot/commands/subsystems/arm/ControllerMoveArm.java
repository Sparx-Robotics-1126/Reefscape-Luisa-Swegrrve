package frc.robot.commands.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ControllerMoveArm extends Command {

    private final DoubleSupplier m_power;
    private final ArmSubsystem m_arm;

    public ControllerMoveArm(DoubleSupplier power, ArmSubsystem arm) {
        addRequirements(RobotContainer.m_arm);
        m_power = power;
        m_arm = arm;
    }

    @Override
    public void execute() {
        // System.out.println("rrrrrrrrrrrrrr");
        double speed = MathUtil.applyDeadband(m_power.getAsDouble(), .1);
        m_arm.moveArm(speed);
    }
}
