package org.team1126.robot.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.Tunable;
import org.team1126.lib.util.Tunable.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants.AlgaeConstants;
import org.team1126.robot.subsystems.ClimbSubsystem.ClimberPosition;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class AlgaeAcquisition extends GRRSubsystem {

    public static enum AlgaePosition {
        kHome(0),
        kOut(30);
        

        private final TunableDouble position;

        private AlgaePosition(double position) {
            this.position = Tunable.doubleValue("algae/positions/" + name(), position);
        }
        
        public double position() {
            return position.value();
        }
    }

    private SparkMax algaeWheels;
    private SparkMax algaeRotation; 

    private RelativeEncoder rotationEncoder;
    
    private SparkMaxConfig wheelConfig;
    private SparkMaxConfig rotationConfig;

    private SparkClosedLoopController rotationController;

    private DigitalInput homeSensor;
    private DigitalInput algaeSensor;

    private double kRotationkS= 0.0;
    private double kRotationkG = .762;
    private double kRotationkV =.762 ;
    private double kRotationkA = 0.0;

    protected ShuffleboardTab algaeTab;

    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    kRotationkS,
                    kRotationkG,
                    kRotationkV,
                    kRotationkA);


    //Creates a new AlgaeAcquisition.
    public AlgaeAcquisition() {
        // if (!RobotBase.isSimulation()){

         algaeWheels = new SparkMax(AlgaeConstants.ALGAE_WHEELS_ID, MotorType.kBrushless);
         algaeRotation = new SparkMax(AlgaeConstants.ALGAE_ROTATION_ID, MotorType.kBrushless);

         homeSensor = new DigitalInput(AlgaeConstants.ALGAE_HOME_ID);
         algaeSensor = new DigitalInput(AlgaeConstants.ALGAE_SENSOR_ID);

         rotationEncoder = algaeRotation.getEncoder();

         rotationController = algaeRotation.getClosedLoopController();

         algaeTab = Shuffleboard.getTab("AlgaeTab");

         wheelConfig = new SparkMaxConfig();
         rotationConfig = new SparkMaxConfig();

        configurePID();
        configureSparkMaxes();
        // }

        }

    /**
     * configure PID settings here.
     */
    private void configurePID(){

        rotationConfig.closedLoop
                .p(.2)
                .i(0)
                .d(0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

        algaeWheels.configure(wheelConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        algaeRotation.configure(rotationConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        
    }

    //for testing
    public void spinAlgaeWheels(double speed) {
        algaeWheels.set(-speed);
    }

    //for testing
    public void moveArm(double speed) {
        algaeRotation.set(speed);
    }

    
    public void reachGoal(double goalDistance){
        rotationController.setReference(goalDistance,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command setGoal(double distance){
        return run(() -> reachGoal(distance));
    }

    /**
     * Returns the position of algae acq
     */
    public double getAngle() {
        if (rotationEncoder == null) {
            return 0;
        }
        return rotationEncoder.getPosition();
    }

    public boolean isAlgaeHome(){
        if (homeSensor == null ) {
            return false;
        }
        return !homeSensor.get();
    }

    public boolean hasAlgae(){
        if (algaeSensor == null) {
            return false;
        }
        return !algaeSensor.get();
    }

    public Command spitAlgae(double speed) {
        return commandBuilder()
        .onExecute(() -> {
            spinAlgaeWheels(speed);})
            .onEnd(() -> spinAlgaeWheels(0));
    }

    public Command spinAlgae(double speed){
        return commandBuilder()
        .onExecute(() -> {
            spinAlgaeWheels(speed);})
            .onEnd(() -> spinAlgaeWheels(0));
    }
         /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the climber is safe to move.
     */
    public Command goTo(AlgaePosition position) {
        return goTo(() -> position);
    }

    public Command acqAlgae(AlgaePosition position, double speed) {
        return commandBuilder("Algae.acquire()")
            .onExecute(() -> {
                spinAlgaeWheels(speed);
                this.reachGoal(position.position());
                // goTo(() -> position);
            })
            .onEnd(() -> {
                if (hasAlgae()) {
                    spinAlgaeWheels(0);
                } 
        });
    }
    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the climber is safe to move.
     */
    private Command goTo(Supplier<AlgaePosition> position) {
        Mutable<Double> holdPosition = new Mutable<>(ClimberPosition.kHome.position());

        return commandBuilder("Algae.goTo()")
            .onInitialize(() -> holdPosition.value = ClimberPosition.kHome.position())
            .onExecute(() -> {
               
                this.reachGoal(position.get().position());
            })
            .onEnd( ()->   moveArm(0));
    }
}
