package frc.robot.commands.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class PulseCommand extends Command {

    private final LEDs ledSubsystem;
    private final Color8Bit color;
    private final int pulseRate;
    private int index;
    private int endIndex;

    public PulseCommand(LEDs ledSubsystem, Color8Bit color, int pulseRate, int startingIndex, int endingIndex) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.pulseRate = pulseRate;
        index = startingIndex;
        addRequirements(ledSubsystem);
        endIndex = endingIndex;
        // halfway is 94
    }

    @Override
    public void execute() {
        ledSubsystem.setPulse(color, pulseRate, index, endIndex);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
