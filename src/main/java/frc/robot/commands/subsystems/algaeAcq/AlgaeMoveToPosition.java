package frc.robot.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAcquisition;

public class AlgaeMoveToPosition extends Command {

    AlgaeAcquisition algaeAcquisition;
    double targetAngle;

    public AlgaeMoveToPosition(AlgaeAcquisition algaeAcquisition, double angle) {
        addRequirements(algaeAcquisition);
        this.algaeAcquisition = algaeAcquisition;
        targetAngle = angle;
    }

    @Override
    public void execute() {
        algaeAcquisition.reachGoal(targetAngle);
        algaeAcquisition.spinAlgaeWheels(.3);
    }

    @Override
    public boolean isFinished() {
        if (algaeAcquisition.hasAlgae()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeAcquisition.spinAlgaeWheels(0);
    }
}
