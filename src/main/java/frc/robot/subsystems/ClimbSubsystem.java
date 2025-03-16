package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private SparkMax climb;

    private RelativeEncoder climbEncoder;

    private SparkMaxConfig climbConfig;

    private SparkClosedLoopController pidController;

    protected ShuffleboardTab climbTab;

    public ClimbSubsystem() {
        // if (!RobotBase.isSimulation()){

        climb = new SparkMax(ClimbConstants.CLIMB_ID, MotorType.kBrushless);
        climbEncoder = climb.getEncoder();
        climbConfig = new SparkMaxConfig();

        pidController = climb.getClosedLoopController();
        climbTab = Shuffleboard.getTab("ClimbTab");
        initShuffleboard();
        configurePID();
        configureSparkMaxes();
        // }
    }

    /*
     * configure PID settings here.
     */
    private void configurePID() {
        climbConfig.closedLoop.p(5).i(0).d(0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        climb.configure(climbConfig, null, null);
    }

    private void initShuffleboard() {
        climbTab.add("Current Climb Position", 0);
    }

    /**
     * Moves the climber at a specific speed
     * @param speed the speed to move the climber
     */
    public void moveClimb(double speed) {
        climb.set(speed);
    }

    /**
     * Returns the current position of the climber
     */
    public double getAngle() {
        return climbEncoder.getPosition();
    }

    public void resetAngle() {
        climbEncoder.setPosition(0);
    }

    public void climbReachGoal(double goalDegree) {
        pidController.setReference(goalDegree, ControlType.kPosition);
    }

    public Command setClimbGoal(double degree) {
        System.out.println("Setting turn goal to " + degree);
        return run(() -> climbReachGoal(degree));
    }

    /**
     * Sets the climber speed to 0
     */
    public void stopTurn() {
        climb.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb position", getAngle());
    }
}
