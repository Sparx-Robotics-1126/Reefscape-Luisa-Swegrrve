package org.team1126.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.team1126.lib.swerve.Perspective;
import org.team1126.lib.swerve.SwerveAPI;
import org.team1126.lib.swerve.SwerveState;
import org.team1126.lib.swerve.config.SwerveConfig;
import org.team1126.lib.swerve.config.SwerveModuleConfig;
import org.team1126.lib.swerve.hardware.SwerveEncoders;
import org.team1126.lib.swerve.hardware.SwerveIMUs;
import org.team1126.lib.swerve.hardware.SwerveMotors;
import org.team1126.lib.util.Alliance;
import org.team1126.lib.util.Math2;
import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.Tunable;
import org.team1126.lib.util.Tunable.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants;
import org.team1126.robot.Constants.FieldConstants;
import org.team1126.robot.Constants.RobotMap;
import org.team1126.robot.util.VisionManager;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0);
    private static final double kTurnRatio = (22.0 / 10.0) * (88.0 / 16.0);
    private static final double kModuleOffset = Units.inchesToMeters(12.25);
    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
    .setName("frontLeft")
    .setLocation(kModuleOffset, kModuleOffset)
    .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kFlMove, true))
    .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kFlTurn, true))
    .setEncoder(SwerveEncoders.canCoder(RobotMap.kFlEncoder, -0.290283, false));

private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
    .setName("frontRight")
    .setLocation(kModuleOffset, -kModuleOffset)
    .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kFrMove, true))
    .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kFrTurn, true))
    .setEncoder(SwerveEncoders.canCoder(RobotMap.kFrEncoder, -0.096924, false));

private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
    .setName("backLeft")
    .setLocation(-kModuleOffset, kModuleOffset)
    .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kBlMove, true))
    .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kBlTurn, true))
    .setEncoder(SwerveEncoders.canCoder(RobotMap.kBlEncoder, 0.353760, false));

private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()

    .setName("backRight")
    .setLocation(-kModuleOffset, -kModuleOffset)
    .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kBrMove, true))
    .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kBrTurn, true))
    .setEncoder(SwerveEncoders.canCoder(RobotMap.kBrEncoder, 0.311035, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.01, 0.02) 
        
                    // Odometry: how fast the odometry updates, lower if we need lower CAN utilization. 
                    // Discretization: Should not be changed.
                    .setMovePID(0.01, 0.0, 0.0)
                    .setMoveFF(0.05, 0.127)
                    .setTurnPID(.05, 0.0, 0.2)
        // .setMovePID(0.08, 0.0, 0.0)  //PID tuning!!!
        // .setMoveFF(0.05, 0.128)         //      |
        // .setTurnPID(0.05, 0.0, 0.2)  //      V
        .setBrakeMode(true, true)  // Think about turning move to false for testing?
        .setLimits(4.0, 0.05, 6, 10.0, 30.0) 

                    // Velocity: the max speed the MOTORS are able to go. please don't change this.
                    // VelDeadband: how much deadband is needed on the controller to start moving. 
                    // SlipAccel: the amount of acceleration the robot can do. the higher the number, the snappier the chnage in direction will be
                    // TorqueAccel: the amount to correct for rotating while at full speed. the higher the number, the more correction.

        .setDriverProfile(4.3, 1.5, 0.15, 4.7, 2.0, 0.15) 

                    // Vel: the max velocity the HUMAN PLAYER can give to the robot.
                    // velExp: the exponential given to the joystick to allow for more sensitive control
                    // velDeadband: self-explanitory
                    // angularVel: limits the angular velocity so that we dont overuse motors
                    // angularVelDeadband: also self-explanatory

        .setPowerProperties(Constants.kVoltage, 60.0, 60.0, 40.0, 40.0)
        .setMechanicalProperties(6.12, 21.4285714286, 0.0, Units.inchesToMeters(3.74))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.pigeon2(RobotMap.kCanandgyro))
        // .setPhoenixFeatures(new CANBus(RobotMap.kSwerveCANBus), false, true, true)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final TunableDouble kTurboSpin = Tunable.doubleValue("swerve/kTurboSpin", 8.0);

    private static final TunableDouble kBeachSpeed = Tunable.doubleValue("swerve/kBeachSpeed", 3.0);
    private static final TunableDouble kBeachTolerance = Tunable.doubleValue("swerve/kBeachTolerance", 0.15);

    private static final TunableDouble kReefAssistKp = Tunable.doubleValue("swerve/kReefAssistKp", 15.0);
    private static final TunableDouble kAutoDriveTolerance = Tunable.doubleValue("swerve/kAutoDriveTolerance", 0.3);
    private static final TunableDouble kReefAssistTolerance = Tunable.doubleValue("swerve/kReefAssistTolerance", 1.3);
    private static final TunableDouble kFacingReefTolerance = Tunable.doubleValue("swerve/kFacingReefTolerance", 1.0);
    private static final TunableDouble kReefDangerDistance = Tunable.doubleValue("swerve/kReefDangerDistance", 0.6);
    private static final TunableDouble kReefHappyDistance = Tunable.doubleValue("swerve/kReefHappyDistance", 0.7);


    private final SwerveAPI api;
    private final SwerveState state;
    private final VisionManager vision;



    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    public final PIDController reefPIDx;
    public final PIDController reefPIDy;
    
    private final PIDController autoPIDangular;

    private final ProfiledPIDController angularPID;

    private final Debouncer dangerDebounce = new Debouncer(0.2);
    private final ReefAssistData reefAssist = new ReefAssistData();

    private Pose2d autoLast = null;
    private Pose2d autoNext = null;
    private Pose2d reefReference = Pose2d.kZero;
    private boolean facingReef = false;
    private double wallDistance = 0.0;

    public Swerve() {
        api = new SwerveAPI(kConfig);
        state = api.state;
        vision = VisionManager.getInstance();


        reefPIDx = new PIDController(6.0, 0.0, 0.40);
        reefPIDy = new PIDController(6.0, 0.0, 0.40);

        autoPIDx = new PIDController(3.3, 0.0, 0.0);
        autoPIDy = new PIDController(3.3, 0.0, 0.0);
        autoPIDangular = new PIDController(14.8, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        angularPID = new ProfiledPIDController(10.0, 0.5, 0.25, new Constraints(10.0, 30.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);
        angularPID.setIZone(0.8);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);
        Tunable.pidController("swerve/angularPID", angularPID);
        Tunable.pidController("swerve/reefPID",reefPIDx);
        Tunable.pidController("swerve/reefPID",reefPIDy);
    }

    @Override
    public void periodic() {
        // Refresh the swerve API.
        api.refresh();

        // Apply vision estimates to the pose estimator.
        api.addVisionMeasurements(vision.getUnreadResults(state.poseHistory));

        // Calculate helpers
        Translation2d reefCenter = Alliance.isBlue() ? FieldConstants.kReefCenterBlue : FieldConstants.kReefCenterRed;
        Translation2d reefTranslation = state.translation.minus(reefCenter);
        Rotation2d reefAngle = new Rotation2d(
            Math.floor(
                reefCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math2.kSixthPi)).getRadians() /
                Math2.kThirdPi
            ) *
            Math2.kThirdPi
        );

        // Save the current alliance's reef location, and the rotation
        // to the reef wall relevant to the robot's position.
        reefReference = new Pose2d(reefCenter, reefAngle);

        // If the robot is rotated to face the reef, within an arbitrary tolerance.
        facingReef = Math2.epsilonEquals(
            0.0,
            reefAngle.minus(state.rotation).getRadians(),
            kFacingReefTolerance.value()
        );

        // Calculate the distance from the robot's center to the nearest reef wall face.
        wallDistance = Math.max(
            0,
            reefAngle.rotateBy(Rotation2d.kPi).minus(reefTranslation.getAngle()).getCos() * reefTranslation.getNorm() -
            FieldConstants.kReefCenterToWallDistance
        );

        SmartDashboard.putBoolean("happyGOose", inRange());
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
    }

    /**
     * Returns the directionless measured velocity of the robot, in m/s.
     */
    @NotLogged
    public double getVelocity() {
        return state.velocity;
    }

    /**
     * Returns true if the goose is happy!! 
     * (Robot is facing the reef and within the happy distance).
     */
    public boolean inRange() {
        return facingReef && wallDistance < kReefHappyDistance.value();
    }

    /**
     * Returns true if the elevator and goose neck are safe
     * to move, based on the robot's position on the field.
     */
    public boolean wildlifeConservationProgram() {
        return dangerDebounce.calculate(wallDistance > kReefDangerDistance.value());
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.kOperator))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            double pitch = state.pitch.getRadians();
            double roll = state.roll.getRadians();


            // var antiBeach = Perspective.kOperator.toPerspectiveSpeeds(
            //     new ChassisSpeeds(
            //         Math.abs(pitch) > kBeachTolerance.value() ? Math.copySign(kBeachSpeed.value(), pitch) : 0.0,
            //         Math.abs(roll) > kBeachTolerance.value()  ? Math.copySign(kBeachSpeed.value(), -roll) : 0.0,
            //         0.0
            //     ),
            //     state.rotation
            // );
            var antiBeach = Perspective.kOperator.toPerspectiveSpeeds(
                new ChassisSpeeds(0.0,0.0,
                    0.0
                ),
                state.rotation
            );
            api.applyAssistedDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                antiBeach,
                Perspective.kOperator,
                true,
                true
            );
        });
    }

    /**
     * SPIN FAST RAHHHHHHH
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command turboSpin(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        Mutable<Double> configured = new Mutable<>(0.0);
        return drive(x, y, angular)
            .beforeStarting(() -> {
                configured.value = api.config.driverAngularVel;
                api.config.driverAngularVel = kTurboSpin.value();
            })
            .finallyDo(() -> api.config.driverAngularVel = configured.value);
    }

    /**
     * Drives the robot using driver input while facing the reef,
     * and "pushing" the robot to center on the selected pipe.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     */
    public Command driveReef(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular, BooleanSupplier left) {
        return commandBuilder("Swerve.driveReef()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);
            })
            .onExecute(() -> {
                double xInput = x.getAsDouble();
                double yInput = y.getAsDouble();
                double angularInput = angular.getAsDouble();
                double norm = Math.hypot(-yInput, -xInput);
                boolean inDeadband = norm < api.config.driverVelDeadband;

                reefAssist.targetPipe = new Pose2d(
                    reefReference
                        .getTranslation()
                        .plus(
                            new Translation2d(
                                FieldConstants.kPipeOffsetX.value(),
                                (left.getAsBoolean() ? FieldConstants.kPipeOffsetYLeft.value() : FieldConstants.kPipeOffsetYRight.value()) * (left.getAsBoolean() ? -1.0 : 1.0)
                            ).rotateBy(reefReference.getRotation())
                        ),
                    reefReference.getRotation()
                   
                );
                Rotation2d robotAngle = reefAssist.targetPipe.getTranslation().minus(state.translation).getAngle();
                Rotation2d xyAngle = !inDeadband
                    ? new Rotation2d(-yInput, -xInput).rotateBy(Alliance.isBlue() ? Rotation2d.kZero : Rotation2d.kPi)
                    : robotAngle;

                double stickDistance = Math.abs(
                    (Math.cos(xyAngle.getRadians()) * (reefAssist.targetPipe.getY() - state.pose.getY()) -
                        Math.sin(xyAngle.getRadians()) * (reefAssist.targetPipe.getX() - state.pose.getX()))
                );

                reefAssist.running =
                    Math2.epsilonEquals(0.0, stickDistance, kReefAssistTolerance.value()) &&
                    Math2.epsilonEquals(0.0, robotAngle.minus(xyAngle).getRadians(), Math2.kHalfPi) &&
                    !inDeadband;

                reefAssist.error = robotAngle.minus(reefReference.getRotation()).getRadians();
                reefAssist.output = reefAssist.running ? reefAssist.error * norm * norm * kReefAssistKp.value() : 0.0;

                double ddd = angularPID.calculate(
                    state.rotation.getRadians(),
                    reefReference.getRotation().getRadians());

               if(reefAssist.targetPipe.getTranslation().getDistance(state.translation) < kAutoDriveTolerance.value() || DriverStation.isAutonomous()) {
                var assist =
                        new ChassisSpeeds(
                            (Alliance.isRed() ? -1 : 1) * reefPIDx.calculate(state.pose.getX(), reefAssist.targetPipe.getX()),
                            (Alliance.isRed() ? -1 : 1) * reefPIDy.calculate(state.pose.getY(), reefAssist.targetPipe.getY()),
                        ddd
                        )
                ;

                api.applyAssistedDriverInput(0.0, 0.0, 0.0, assist, Perspective.kOperator, true, true);
               } else {
                var assist = Perspective.kOperator.toPerspectiveSpeeds(
                    new ChassisSpeeds(
                        0.0,
                        reefAssist.output,
                        ddd
                    ),
                    reefReference.getRotation()
                );


                api.applyAssistedDriverInput(xInput, yInput, angularInput, assist, Perspective.kOperator, true, true);
               }

                
            })
            .onEnd(() -> reefAssist.running = false);
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Resets the autonomous trajectory following PID controllers. This
     * command does not require the swerve subsystem, and can be safely
     * composed in parallel with another swerve command.
     */
    public Command resetAutoPID() {
        return Commands.runOnce(() -> {
            autoPIDx.reset();
            autoPIDy.reset();
            autoPIDangular.reset();
        })
            .ignoringDisable(true)
            .withName("Swerve.resetAutoPID");
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement. This
     * method is not intended for use outside of creating an {@link AutoFactory}.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        api.resetPose(pose);
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        autoLast = autoNext;
        autoNext = sample.getPose();

        Pose2d pose = state.pose;
        api.applySpeeds(
            new ChassisSpeeds(
                sample.vx + autoPIDx.calculate(pose.getX(), sample.x),
                sample.vy + autoPIDy.calculate(pose.getY(), sample.y),
                sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading)
            ),
            Perspective.kBlue,
            true,
            false
        );
    }

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }
}
