package org.team1126.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private int chaseIndex = 0;

    public LEDs(int port, int ledCount) {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();
    }

    public void setColor(int index, int red, int green, int blue) {
        if (index < 0 || index >= ledBuffer.getLength()) {
            return;
        }
        ledBuffer.setRGB(index, red, green, blue);
    }

    public void setChaseColor(Color8Bit color, int length) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if ((i + chaseIndex) % length == 0) {
                ledBuffer.setLED(i, color);
            } else {
                ledBuffer.setLED(i, new Color8Bit(0, 0, 0));
            }
        }
        chaseIndex = (chaseIndex + 1) % length;
        update();
    }
public void setRainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        final int hue = (i * 180 / ledBuffer.getLength() + chaseIndex) % 180;
        ledBuffer.setHSV(i, hue, 255, 128);
    }
    chaseIndex = (chaseIndex + 1) % 180;
    update();
}


public void setBlink(Color8Bit color, int blinkRate) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        if ((chaseIndex / blinkRate) % 2 == 0) {
            ledBuffer.setLED(i, color);
        } else {
            ledBuffer.setLED(i, new Color8Bit(0, 0, 0));
        }
    }
    chaseIndex++;
    update();
}

public void setGradient(Color8Bit startColor, Color8Bit endColor) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        int red = (int) (startColor.red + (endColor.red - startColor.red) * i / (double) ledBuffer.getLength());
        int green = (int) (startColor.green + (endColor.green - startColor.green) * i / (double) ledBuffer.getLength());
        int blue = (int) (startColor.blue + (endColor.blue - startColor.blue) * i / (double) ledBuffer.getLength());
        ledBuffer.setRGB(i, red, green, blue);
    }
    update();
}

public void setPulse(Color8Bit color, int pulseRate, int startIndex, int endIndex) {
    int brightness = (int) (128 + 127 * Math.sin(chaseIndex / (double) pulseRate * 2 * Math.PI));
    for (int i = startIndex; i < endIndex; i++) {
        ledBuffer.setRGB(i, color.red * brightness / 255, color.green * brightness / 255, color.blue * brightness / 255);
    }
    chaseIndex++;
    update();
}


public AddressableLEDBuffer getLedBuffer() {
    return ledBuffer;
}
    public void update() {
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command setAllianceColorCommand() {
        return run(() -> {
            if (DriverStation.getAlliance().get() == Alliance.Blue ) {
                setGradient(new Color8Bit(0, 0, 255), new Color8Bit(0, 255, 0));
            } else {
                setGradient(new Color8Bit(255, 0, 0), new Color8Bit(255, 255, 0));
            }
        });
    }
}