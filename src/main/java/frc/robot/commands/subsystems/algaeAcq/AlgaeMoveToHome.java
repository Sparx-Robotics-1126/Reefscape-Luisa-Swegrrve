package frc.robot.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAcquisition;

public class AlgaeMoveToHome extends Command {

    AlgaeAcquisition algaeAcquisition;
    double targetAngle = 0;

    public AlgaeMoveToHome(AlgaeAcquisition algaeAcquisition) {
        addRequirements(algaeAcquisition);
        this.algaeAcquisition = algaeAcquisition;
    }

    @Override
    public void execute() {
        algaeAcquisition.reachGoal(targetAngle);
    }

    @Override
    public boolean isFinished() {
        if (algaeAcquisition.getAngle() <= 0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeAcquisition.spinAlgaeWheels(0);
    }
}
