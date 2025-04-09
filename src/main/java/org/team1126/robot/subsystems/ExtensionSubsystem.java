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

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team1126.lib.util.Math2;
import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.Tunable;
import org.team1126.lib.util.Tunable.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants.ArmConstants;
import org.team1126.robot.Robot;


public final class ExtensionSubsystem extends GRRSubsystem {

   
    public static enum ExtensionPosition {
        kHome(0.01),
        kCoralStation(0.01),
        kLevel1(0.013659),
        kLevel2(-0.1431989),
        kLevel3(-0.2),
        kLevel4(-0.55);

        private final TunableDouble position;

        private ExtensionPosition(double position) {
            this.position = Tunable.doubleValue("extension/positions/" + name(), position);
        }

        public double position() {
            return position.value();
        }

        private static ExtensionPosition closeTo(double position) {
            ExtensionPosition closest = null;
            double min = Double.MAX_VALUE;
            for (ExtensionPosition option : values()) {
                double distance = Math.abs(option.position() - position);
                if (Math2.epsilonEquals(0.0, distance, kCloseToTolerance.value()) && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }


    private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("extension/kCloseToTolerance", 0.35);
    private static final TunableDouble kZeroTolerance = Tunable.doubleValue("extension/kZeroTolerance", 0.15);

    private SparkMax extension;
    private final Robot robot;
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

    public ExtensionSubsystem(Robot robot) {
        this.robot = robot;
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
  /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    public Command goTo(ExtensionPosition position, BooleanSupplier safe) {
        return goTo(() -> position, () -> 0.0, safe);
    }
    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    private Command goTo(Supplier<ExtensionPosition> position, DoubleSupplier fudge, BooleanSupplier safe) {
        Mutable<Double> holdPosition = new Mutable<>(ExtensionPosition.kHome.position());

        return commandBuilder("Extension.goTo()")
            .onInitialize(() -> holdPosition.value = ExtensionPosition.kHome.position())
            .onExecute(() -> {
                double target = position.get().position();
                double currentPosition = getExtension();

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        ExtensionPosition close = ExtensionPosition.closeTo(currentPosition);
                        holdPosition.value = close != null ? close.position() : currentPosition;
                    }

                    target = holdPosition.value;
                } else {
                    holdPosition.value = ExtensionPosition.kHome.position();
                }

                
                if (currentPosition - kZeroTolerance.value() <= 0.0 && target - kZeroTolerance.value() <= 0.0) {
                    this.stopExtension();
                } else {
                    this.extReachGoal(targetExtension);
                }
            });
    }
}
