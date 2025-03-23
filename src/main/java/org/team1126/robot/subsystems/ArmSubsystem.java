package org.team1126.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
// import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {

    /**
     * Subsystem-wide setpoints
     */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }


    private SparkMax turnMotor;
    private SparkMax turnFollower; // follows turnMotor
    private SparkClosedLoopController turnController;

    private DigitalInput homeSensor;

    private RelativeEncoder turnEncoder;

    private SparkMaxConfig turnConfig;
    private SparkMaxConfig turn2Config;

    protected ShuffleboardTab armTab;

    private double targetAngle;

    public boolean isL1;
    public boolean isL2;
    public boolean isL3;
    public boolean isL4;

    ElevatorFeedforward m_feedforward =
    new ElevatorFeedforward(
       0.0,
       .762,
        .762,
       0);

//     ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.1, 0.1);

    public ArmSubsystem() {
        //   if (!RobotBase.isSimulation()){

        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        
        turnController = turnMotor.getClosedLoopController();

        turnEncoder = turnMotor.getEncoder();

        turnConfig = new SparkMaxConfig();

        homeSensor = new DigitalInput(ArmConstants.ARM_SENSOR_ID);

        turn2Config = new SparkMaxConfig();
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

        var p=.03;
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(p)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                .velocityFF(1.0/5767);
                // .minOutput(-.04).maxOutput(.06)

                turn2Config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(p)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                .velocityFF(1.0/5767);

    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

//        extensionConfig.encoder
//                        .positionConversionFactor(Elevatore)


        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turnFollower.configure(turn2Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        turnMotor.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    private void initShuffleboard() {

    }

    public void moveArm(double speed) {
        // System.out.println("ddddd");
        turnMotor.set(speed);
    }

    /**
     * Returns the position of the arm
     */
    public double getArmAngle() {
        if (turnEncoder == null) {
            return 0;
        }
        return turnEncoder.getPosition();
    }

    
    public void turnReachGoal(double goalDegree) {
        // System.out.println("In here " + goalDegree);
        turnController.setReference(goalDegree, ControlType.kPosition);
    }

    public Command setTurnGoal(double degree) {
        // System.out.println("Setting turn goal to " + degree);
        return run(() -> turnReachGoal(degree));
    }

    

    /**
     * Sets arm speed to 0
     */
    public void stopTurn() {
        turnMotor.set(0.0);
    }

    public boolean isHome(){
        return !homeSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm position", getArmAngle());
        SmartDashboard.putNumber("Target Positon", targetAngle);
    }

}
