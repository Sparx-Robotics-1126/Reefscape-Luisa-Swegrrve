package frc.robot.commands.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class TeamLights extends Command {

    private LEDs ledSubsystem;

    public TeamLights(LEDs ledSubsystem) {
        addRequirements(ledSubsystem);
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize() {
        if (!DriverStation.getAlliance().isPresent()) {
            return;
        }
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            for (int i = 0; i < ledSubsystem.getLedBuffer().getLength(); i++) {
                ledSubsystem.setColor(i, 0, 0, 255);
            }
        } else {
            for (int i = 0; i < ledSubsystem.getLedBuffer().getLength(); i++) {
                ledSubsystem.setColor(i, 255, 0, 0);
            }
        }
        ledSubsystem.update();
    }

    @Override
    public boolean isFinished() {
        return true; // Command completes immediately after setting the color
    }
}
