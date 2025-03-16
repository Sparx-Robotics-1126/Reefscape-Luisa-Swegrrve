package frc.robot.commands.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class RainbowCommand extends Command {

    private final LEDs ledSubsystem;

    public RainbowCommand(LEDs ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setRainbow();
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
