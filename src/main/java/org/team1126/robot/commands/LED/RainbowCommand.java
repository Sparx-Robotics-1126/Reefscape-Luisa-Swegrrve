package org.team1126.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.LEDs;

public class RainbowCommand extends Command {
    private final LEDs ledSubsystem;
    private final TeamLights lights;

    public RainbowCommand(LEDs ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
        lights = new TeamLights(ledSubsystem);
        
    }

    @Override
    public void execute() {
        ledSubsystem.setRainbow();
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }

    @Override
    public void end(boolean interrupted){
        lights.initialize();
    }
}
