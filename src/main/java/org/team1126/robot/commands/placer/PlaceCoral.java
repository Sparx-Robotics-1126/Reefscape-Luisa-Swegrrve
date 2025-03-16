package org.team1126.robot.commands.placer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.subsystems.PlacerSubsystem;

public class PlaceCoral extends Command {
    
    private PlacerSubsystem placer;
    private double speed;
    
    public PlaceCoral(PlacerSubsystem placer, double speed) {
        addRequirements(placer);
        this.placer = placer;
        this.speed = speed;
    }
    
    @Override
    public void execute() {
        placer.movePlacer(speed);
    }

    @Override
    public void end(boolean interruped){
        placer.movePlacer(0);
    }
}
