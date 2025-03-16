package org.team1126.robot.commands.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.LEDs;

public class GradientCommand extends Command {
    private final LEDs ledSubsystem;
    private final Color8Bit startColor;
    private final Color8Bit endColor;

    public GradientCommand(LEDs ledSubsystem, Color8Bit startColor, Color8Bit endColor) {
        this.ledSubsystem = ledSubsystem;
        this.startColor = startColor;
        this.endColor = endColor;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setGradient(startColor, endColor);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
