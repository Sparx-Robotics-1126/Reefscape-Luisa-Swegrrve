// package org.team1126.robot.commands.placer;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import org.team1126.robot.subsystems.PlacerSubsystem;

// public class AnalogPlacer extends Command {

//      private final DoubleSupplier m_power;
//     private final PlacerSubsystem m_placer;
//     private boolean isReverse;

//     public AnalogPlacer(DoubleSupplier power, PlacerSubsystem arm, boolean reverse) {
//         addRequirements(arm);
//         m_power = power;
//         m_placer = arm;
//         isReverse = reverse;
//     }

//     @Override
//     public void execute() {
//         double speed;
//         if(isReverse){
//             speed =  -MathUtil.applyDeadband(m_power.getAsDouble(), .02) * .3;
//         }else {
//             speed =  MathUtil.applyDeadband(m_power.getAsDouble(), .02) * .3;
//         }
       
//         m_placer.movePlacer(speed);
//     }

//     @Override
//     public void end(boolean interruped){
//         m_placer.movePlacer(0);
//     }

// }
