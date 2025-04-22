package org.team1126.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants.PlacerConstants;


/**
 * PlacerSubsystem is responsible for the device attached to the end of the 
 * ExtensionSubsystem that is responsible for hanging on to and releasing 
 * coral when it is time to score / drop it.
 * 
 * Considerations:
 * - logic to recieve coral should have more bounce to it
 * - if possible a top and bottom sensor should be reintroduced
 * - pid should be applied to the motors on the placer
 * - tunables should be added where possible
 */
public class PlacerSubsystem extends GRRSubsystem {

    private SparkMax placer;
    private SparkMax placerFollower;
    private SparkMaxConfig placerConfig;
    private SparkMaxConfig placerFollowerConfig;
    private CANrange bottomSensor;
    private CANrangeConfigurator bottomSensorConfig;
    private Trigger hasGamepiece;

    private final Debouncer debouncer = new Debouncer(0.14, DebounceType.kRising);


    /**
     * Default constructor. Following instantiation this constructor will make
     * a call to the configureSparkMaxes() method, thus it does not need to be
     * called separately.
     * 
     * TODO: consider pulling the configure SparkMaxes() method into the
     * constructor unless it has the potential of being called multiple times
     */
    public PlacerSubsystem() { 
        placer = new SparkMax(PlacerConstants.PLACER_ID, MotorType.kBrushless);
        placerFollower = new SparkMax(PlacerConstants.PLACER_FOLLOWER_ID, MotorType.kBrushless);

        placerConfig = new SparkMaxConfig();
        placerFollowerConfig = new SparkMaxConfig();

        bottomSensor = new CANrange(36);
        bottomSensorConfig = bottomSensor.getConfigurator();
        hasGamepiece =  new Trigger(this::bottomHasCoral).debounce(0);
        
        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        CANrangeConfiguration config = new CANrangeConfiguration();
        ProximityParamsConfigs proxConfig = new ProximityParamsConfigs();
        config.ProximityParams.ProximityThreshold = 0.00000001;
        config.FovParams.FOVRangeX = 7;
        config.FovParams.FOVRangeY = 7;
        proxConfig.MinSignalStrengthForValidMeasurement = 15000;
        placerFollowerConfig.follow(PlacerConstants.PLACER_ID,true);
        placer.configure(placerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        placerFollower.configure(placerFollowerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        bottomSensorConfig.apply(config);
        bottomSensorConfig.apply(proxConfig);
    }

    /**
     * Release the coral!
     * 
     * @return a command that will release the coral!
     */
    public Command releaseCoral() {
        return commandBuilder("PlacerSubsystem.releaseCoral()").onExecute(() ->
            placer.set(-1) // Magic numbers!!!
        )
        .isFinished(true)
        .ignoringDisable(false);
    }
    
public Command ingestCoral(){

    return commandBuilder("PlacerSubsystem.ingestCoral()").onExecute(() ->
    placer.set(.15)) 
    .isFinished(() -> { System.out.println(bottomHasCoral());
        if (bottomHasCoral()){
            return true;
        }
        return false;
        }
    )
    .onEnd(() -> placer.set(0));
}

public Command positionCoral(){
    System.out.println("HERE");
    return commandBuilder("PlacerSubsystem.ingestCoral()").onExecute(() ->
    placer.set(-.1))
    .isFinished(() -> {
        if (!bottomHasCoral()){
            return true;
        }
        return false;
        }
    )
    .onEnd(() -> placer.set(0));
}


public Command analogPlacer(DoubleSupplier power,boolean reverse){
    return commandBuilder("PlacerSubsystem.analogPlacer").onExecute(()->{
        double speed;
         if(reverse){
            speed =  -MathUtil.applyDeadband(power.getAsDouble(), .02) * .3;
        } else {
            speed =  MathUtil.applyDeadband(power.getAsDouble(), .02) * .3;
        }
       
        placer.set(speed);
    }
    )
    .onEnd(() -> placer.set(0));
}

public Command placeCoral(double speed){
    return commandBuilder("PlacerSubsystem.placeCoral").onExecute(() -> {
        placer.set(speed);
    })
    .onEnd(() ->placer.set(0));
}

    /**
     * Moves the placer at a set speed.
     * 
     * @param speed the speed to move the placer at.
     * @return a command that represents the ability to move the placer.
     */
    public Command movePlacer(DoubleSupplier speed) {
        return commandBuilder("PlacerSubsystem.clearCoral()").onExecute(() ->
            placer.set(speed.getAsDouble())
        )
        .isFinished(true)
        .ignoringDisable(false);
    }

    /**
     * Returns if the bottom sensor sees the coral
     * @return true if the bottom sensor sees the coral, false otherwise
     */
    public boolean bottomHasCoral() {
        return debouncer.calculate(bottomSensor.getIsDetected().getValue());
    }

    /**
     * Gets the speed of the placer.
     * 
     * @return the double representing the speed of the placer.
     */
    public double getSpeed(){
        return placer.get();
    }

    public boolean coralClear() {
        return !bottomSensor.getIsDetected().getValue();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Bottom sensor", bottomHasCoral());
        SmartDashboard.putNumber("Placer distance", bottomSensor.getDistance().getValueAsDouble());
    }
}
