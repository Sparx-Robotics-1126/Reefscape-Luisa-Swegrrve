package frc.robot.commands.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class ChaseLEDColorCommand extends Command {

    private final LEDs ledSubsystem;
    private final Color8Bit color;
    private final int length;

    public ChaseLEDColorCommand(LEDs ledSubsystem, Color8Bit color, int length) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.length = length;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setChaseColor(color, length);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
