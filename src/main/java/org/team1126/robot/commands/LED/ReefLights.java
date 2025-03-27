package org.team1126.robot.commands.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
// import org.team1126.robot.RobotContainer;
import org.team1126.robot.subsystems.LEDs;

public class ReefLights extends Command {
    private LEDs ledSubsystem;
    private boolean isRight;
    private int color;
    private TeamLights lights;

    public ReefLights(LEDs ledSubsystem, boolean isRight, int reefHeight){
        addRequirements(ledSubsystem);
        this.ledSubsystem = ledSubsystem;
        this.isRight = isRight;
        this.color = reefHeight;
        lights = new TeamLights(ledSubsystem);
    }
    
    // halfway is 94

    @Override
    public void initialize() {

        int start = 0;
        int mid = ledSubsystem.getLedBuffer().getLength() / 2;
        int end = ledSubsystem.getLedBuffer().getLength();
        if(isRight){
            if(color == 1){
                for (int i = start; i < end; i++) {
                    ledSubsystem.setColor(i,255, 141, 0);
                }
            } else if(color == 2){
                for (int i = start; i < end; i++) {
                    ledSubsystem.setColor(i,186, 255, 0);
                }
            } else if (color == 3){
                for (int i = start; i < end; i++) {
                    ledSubsystem.setColor(i, 0, 255, 240);
                }
            } else if (color == 4){
                for (int i = start; i < end; i++) {
                    ledSubsystem.setColor(i, 148, 0, 255);
                }
            } else if (color == 5) {
                    ledSubsystem.setRainbow();
                
            }
        // }  else {
        //     if(color == 1){
        //         for (int i = mid; i < end; i++) {
        //             ledSubsystem.setColor(i,255, 141, 0);
        //         }
        //     } else if(color == 2){
        //         for (int i = mid; i < end; i++) {
        //             ledSubsystem.setColor(i,186, 255, 0);
        //         }
        //     } else if (color == 3){
        //         for (int i = mid; i < end; i++) {
        //             ledSubsystem.setColor(i, 0, 255, 240);
        //         }
        //     } else if (color == 4){
        //         for (int i = mid; i < end; i++) {
        //             ledSubsystem.setColor(i, 148, 0, 255);
        //         }
        //     } 
        } else {
            if (!DriverStation.getAlliance().isPresent()) {
                return;
            }
            if(DriverStation.getAlliance().get() == Alliance.Blue) {
                for(int i = 0; i < ledSubsystem.getLedBuffer().getLength(); i++)  {
                    ledSubsystem.setColor(i, 0, 0, 255);
            }
            } else {
                for(int i = 0; i < ledSubsystem.getLedBuffer().getLength(); i++)  {
                    ledSubsystem.setColor(i, 255, 0, 0);
                }
        }
        }
        //System.out.println("Color = " + color);

        ledSubsystem.update();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
       lights.initialize();
    }



}
