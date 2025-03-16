package org.team1126.robot.commands.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.LEDs;


public class BlinkCommand extends Command {
    private final LEDs ledSubsystem;
    private final Color8Bit color;
    private final int blinkRate;

    public BlinkCommand(LEDs ledSubsystem, Color8Bit color, int blinkRate) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.blinkRate = blinkRate;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setBlink(color, blinkRate);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
