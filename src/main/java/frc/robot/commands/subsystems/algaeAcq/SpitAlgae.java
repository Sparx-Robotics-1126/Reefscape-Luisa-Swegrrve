package frc.robot.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAcquisition;

public class SpitAlgae extends Command {

    private AlgaeAcquisition algae;

    public SpitAlgae(AlgaeAcquisition algae) {
        addRequirements(algae);
        this.algae = algae;
    }

    @Override
    public void execute() {
        algae.spinAlgaeWheels(-4);
    }

    @Override
    public boolean isFinished() {
        if (!algae.hasAlgae()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algae.spinAlgaeWheels(0);
    }
}
