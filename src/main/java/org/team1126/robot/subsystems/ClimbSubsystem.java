package org.team1126.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.Tunable;
import org.team1126.lib.util.Tunable.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends GRRSubsystem {

    public static enum ClimberPosition {
        kHome(0),
        kOut(-112.55),
        kIn(125);
        

         private final TunableDouble position;

            private ClimberPosition(double position) {
            this.position = Tunable.doubleValue("climber/positions/" + name(), position);
        }

        public double position() {
            return position.value();
        }

        // private static ClimberPosition closeTo(double position) {
        //     ClimberPosition closest = null;
        //     double min = Double.MAX_VALUE;
        //     for (ClimberPosition option : values()) {
        //         double distance = Math.abs(option.position() - position);
        //         if (Math2.epsilonEquals(0.0, distance, kCloseToTolerance.value()) && distance <= min) {
        //             closest = option;
        //             min = distance;
        //         }
        //     }

        //     return closest;
        // }
    }

    // private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("climber/kCloseToTolerance", 0.35);
    // private static final TunableDouble kZeroTolerance = Tunable.doubleValue("climber/kZeroTolerance", 0.15);

    private SparkMax climb;

    private RelativeEncoder climbEncoder;

    private SparkMaxConfig climbConfig;

    private SparkClosedLoopController pidController;

    private boolean beach;

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
    private void configurePID(){
        climbConfig.closedLoop
                .p(5)
                .i(0)
                .d(0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);        
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        climb.configure(climbConfig, null, null);
    }

    private void initShuffleboard(){

        climbTab.add("Current Climb Position",0);
    }
    
    /**
     * Moves the climber at a specific speed
     * @param speed the speed to move the climber
     */
    public void moveClimb(double speed){
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

    public void setBeachMode(boolean set) {
        beach = set;
    }

    public boolean getBeachMode(){
        return beach;
    }


     /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the climber is safe to move.
     */
    public Command goTo(ClimberPosition position) {
        return goTo(() -> position);
    }
    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the climber is safe to move.
     */
    private Command goTo(Supplier<ClimberPosition> position) {
        Mutable<Double> holdPosition = new Mutable<>(ClimberPosition.kHome.position());

        return commandBuilder("Climber.goTo()")
            .onInitialize(() -> holdPosition.value = ClimberPosition.kHome.position())
            .onExecute(() -> {
               
                this.climbReachGoal(position.get().position());
                this.setBeachMode(false);
            })
            .onEnd( ()->   moveClimb(0));
    }
}
