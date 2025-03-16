package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class EndGameRumble extends Command {

    private int startTime = 135 - 20;
    private int endTime = 135 - 19;

    private CommandXboxController controller;

    public EndGameRumble(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void execute() {
        if (DriverStation.getMatchTime() < startTime) {
            controller.setRumble(RumbleType.kLeftRumble, 1);
            controller.setRumble(RumbleType.kRightRumble, 1);
        } else if (DriverStation.getMatchTime() > endTime) {
            controller.setRumble(RumbleType.kLeftRumble, 0);
            controller.setRumble(RumbleType.kRightRumble, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
