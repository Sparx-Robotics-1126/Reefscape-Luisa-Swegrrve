// package frc.robot.commands.subsystems.algaeAcq;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.AlgaeAcquisition;

// public class MoveAlgae extends Command {

//     private DoubleSupplier m_power;
//     private AlgaeAcquisition m_algae;

//     public MoveAlgae(AlgaeAcquisition algae, DoubleSupplier power) {
//         addRequirements(RobotContainer.m_algae);
//         m_algae = algae;
//         m_power = power;
//     }

//     @Override
//     public void execute() {
//         double speed = MathUtil.applyDeadband(m_power.getAsDouble(), .1);
//         m_algae.moveArm(speed);
//     }

// }
