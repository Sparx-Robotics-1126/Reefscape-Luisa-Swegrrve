// package frc.team1126.commands.subsystems.coralAcq;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.team1126.RobotContainer;
// import frc.team1126.subsystems.CoralAcquisition;

// public class AcqMoveOut extends Command {

//     private CoralAcquisition acq;

//     public AcqMoveOut(CoralAcquisition acq) {
//         addRequirements(RobotContainer.m_coralAcq);
//         this.acq = acq;
//     }

//     @Override
//     public void execute() {
//         if(acq.getAngle() < 90) {
//             acq.moveOut();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if(acq.getAngle() >= 90) {
//             return true;
//         }
//         return false;
//     }
// }
