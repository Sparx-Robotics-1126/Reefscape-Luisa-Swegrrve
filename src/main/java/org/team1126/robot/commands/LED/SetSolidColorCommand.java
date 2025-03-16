package org.team1126.robot.commands.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.LEDs;

public class SetSolidColorCommand extends Command {
    private final LEDs ledSubsystem;
    private final Color8Bit color;

    public SetSolidColorCommand(LEDs ledSubsystem, Color8Bit color) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getLedBuffer().getLength(); i++) {
            ledSubsystem.setColor(i, color.red, color.green, color.blue);
        }
        ledSubsystem.update();
    }

    @Override
    public boolean isFinished() {
        return true; // Command completes immediately after setting the color
    }
}
