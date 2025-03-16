package org.team1126.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1126.robot.Constants.ArmConstants;

public class ExtensionSubsystem extends SubsystemBase {

    /** Subsystem-wide setpoints */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    private SparkMax extension;
    private SparkClosedLoopController extensionController;

    private RelativeEncoder extensionEncoder;

    private SparkMaxConfig extensionConfig;

    private DigitalInput homeSensor;

    protected ShuffleboardTab armTab;

    private double kElevatorkS= 0.0;
    private double kElevatorkG = .762;
    private double kElevatorkV =.762 ;
    private double kElevatorkA = 0.0;

    private double targetExtension;


    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    kElevatorkS,
                    kElevatorkG,
                    kElevatorkV,
                    kElevatorkA);

    public ExtensionSubsystem() {
//   if (!RobotBase.isSimulation()){

        extension = new SparkMax(ArmConstants.ELEVATOR_ID, MotorType.kBrushless);
        extensionController = extension.getClosedLoopController();
        extensionEncoder = extension.getEncoder();

        extensionConfig = new SparkMaxConfig();

        homeSensor = new DigitalInput(ArmConstants.EXTENSION_SENSOR_ID);

        armTab = Shuffleboard.getTab("ArmTab");

        initShuffleboard();
        configurePID();
        configureSparkMaxes();
//   }

    }

    /*
     * configure PID settings here.
     */
    private void configurePID() {


        extensionConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(3.1)
                .i(0)
                .d(0)
                .outputRange(-.25, .25)
                .velocityFF(1.0/5767);
    }
    
    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

//        extensionConfig.encoder
//                        .positionConversionFactor(Elevatore)

        extension.configure(extensionConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    private void initShuffleboard(){
        

    }

    /**
     * Moves the extension to home
     */
    public void extensionToHome() {
        if(extensionEncoder.getPosition() > 0) {
            extension.set(-0.5);
        }
    }

    /**
     * Returns the current position of the extension 
     */
    public double getExtension() {
        if (extensionEncoder == null) {
            return 0;
        }
        return extensionEncoder.getPosition();
    }

    public void extReachGoal(double goalDistance){
        extensionController.setReference(goalDistance, ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward.calculate(goalDistance));
        targetExtension = goalDistance;
    }

    public Command setExtGoal(double distance){
        return run(() -> extReachGoal(distance));
    }

    /**
     * Sets the speed of the extension to 0
     */
    public void stopExtension() {
        extension.set(0.0);
    }

    public boolean isExtensionHome(){
        return !homeSensor.get();
    }

    public void moveExtension(double speed){
        extension.set(speed);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Extension Position", getExtension());
        SmartDashboard.putNumber("Target Positon", targetExtension);
    
       
    }

    
}
