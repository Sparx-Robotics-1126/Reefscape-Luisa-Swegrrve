// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package org.team1126.robot;

// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.cscore.HttpCamera;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import org.team1126.robot.Constants.AprilTagPositions;
// import org.team1126.robot.Constants.ArmConstants;
// import org.team1126.robot.Constants.OperatorConstants;
// import org.team1126.robot.commands.LED.RainbowCommand;
// import org.team1126.robot.commands.LED.ReefLights;
// import org.team1126.robot.commands.LED.TeamLights;
// import org.team1126.robot.commands.arm.ControllerMoveArm;
// // import frc.robot.commands.subsystems.algaeAcq.AlgaeMoveToHome;
// // import frc.robot.commands.subsystems.algaeAcq.AlgaeMoveToPosition;
// // import frc.robot.commands.subsystems.algaeAcq.SpitAlgae;
// // import org.team1126.robot.commands.subsystems.arm.ControllerMoveArm;
// import org.team1126.robot.commands.arm.MoveArmToAngle;
// import org.team1126.robot.commands.arm.MoveExtHome;
// import org.team1126.robot.commands.arm.MoveExtensionToPos;
// import org.team1126.robot.commands.climb.ClimbMoveToPos;
// import org.team1126.robot.commands.placer.AcquireCoral;
// import org.team1126.robot.commands.placer.AnalogPlacer;
// import org.team1126.robot.commands.placer.IngestCoral;
// import org.team1126.robot.commands.placer.PlaceCoral;
// import org.team1126.robot.commands.placer.PositionCoral;
// // import frc.robot.subsystems.AlgaeAcquisition;
// import org.team1126.robot.subsystems.ArmSubsystem;
// import org.team1126.robot.subsystems.ClimbSubsystem;
// import org.team1126.robot.subsystems.ExtensionSubsystem;
// import org.team1126.robot.subsystems.LEDs;
// import org.team1126.robot.subsystems.PlacerSubsystem;
// import org.team1126.robot.subsystems.Swerve;

// // import org.team1126.robot.subsystems.swervedrive.SwerveSubsystem;
// import java.io.File;
// // import frc.lib.swervelib.SwerveInputStream;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
//  * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
//  * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer {

//     // public static final ArmSubsystem m_arm = new ArmSubsystem();
    
//     // public static final ExtensionSubsystem m_extension = new ExtensionSubsystem();
    
//     // public static final AlgaeAcquisition m_algae = new AlgaeAcquisition();

//     // public static final ClimbSubsystem m_climb = new ClimbSubsystem();

//     // public static final PlacerSubsystem m_placer = new PlacerSubsystem();

//     public static CommandXboxController driver = new CommandXboxController(0);
//     // public static CommandXboxController m_operator = new CommandXboxController(1);

//     // public static final LEDs ledSubsystem = new LEDs(0, 200); //PORT IS PWM!!!

//     // public static HttpCamera colorCamera;

//     // final static SendableChooser<Command> m_chooser = new SendableChooser<>();
    
//   // The robot's subsystems and commands are defined here...
// //   final static Swerve swerve ;// = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
// //                                                                                 "swerve/neo"));
// public final Swerve swerve;
//   /**
//    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
//    */
//   // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerve.getSwerveDrive(),
//   //                                                               () -> m_driver.rightTrigger().getAsBoolean() ? m_driver.getLeftY() * -0.25 : m_driver.getLeftY() * -1,
//   //                                                               () -> m_driver.rightTrigger().getAsBoolean() ? m_driver.getLeftX() * -0.25 : m_driver.getLeftX() * -1)
//   //                                                           .withControllerRotationAxis(() -> m_driver.getRightX() * -1 )
//   //                                                           .deadband(OperatorConstants.DEADBAND)
//   //                                                           .scaleTranslation(0.8)
//   //                                                           // .scaleRotation(0.4)
//   //                                                           .allianceRelativeControl(true);       

//   /**
//    * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
//    */
//   // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driver::getRightX,
//   //                                                                                            m_driver::getRightY)
//   //                                                          .headingWhile(true);

//   /**
//    * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
//    */
//   // SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
//   //                                                            .allianceRelativeControl(false);

//   // SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerve.getSwerveDrive(),
//   //                                                                       () -> -m_driver.getLeftY(),
//   //                                                                       () -> -m_driver.getLeftX())
//   //                                                                   .withControllerRotationAxis(() -> m_driver.getRawAxis(
//   //                                                                       2))
//   //                                                                   .deadband(OperatorConstants.DEADBAND)
//   //                                                                   .scaleTranslation(0.8)
//   //                                                                   .allianceRelativeControl(true);
//   // // Derive the heading axis with math!
//   // SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
//   //                                                                              .withControllerHeadingAxis(() ->
//   //                                                                                                             Math.sin(
//   //                                                                                                                 m_driver.getRawAxis(
//   //                                                                                                                     2) *
//   //                                                                                                                 Math.PI) *
//   //                                                                                                             (Math.PI *
//   //                                                                                                              2),
//   //                                                                                                         () ->
//   //                                                                                                             Math.cos(
//   //                                                                                                                 m_driver.getRawAxis(
//   //                                                                                                                     2) *
//   //                                                                                                                 Math.PI) *
//   //                                                                                                             (Math.PI *
//   //                                                                                                              2))
//   //                                                                              .headingWhile(true)
//   //                                                                              .translationHeadingOffset(true)
//   //                                                                              .translationHeadingOffset(Rotation2d.fromDegrees(
//   //                                                                                  0));

//   /**
//    * The container for the robot. Contains subsystems, OI devices, and commands.
//    */
//   public RobotContainer()
//   {
//     // Configure the trigger bindings
//     DriverStation.silenceJoystickConnectionWarning(true);

//     swerve = new Swerve();


//     // NamedCommands.registerCommand("test", Commands.print("I EXIST"));

//             // configurePathPlanner();

//             // colorCamera = new HttpCamera("Color Camera", "http://10.11.26.11");

//         // human control for climb and algae

//         // m_climb.setDefaultCommand(new ClimbMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftX.value), m_climb));
//         // m_arm.setDefaultCommand(new ControllerMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_arm));
//         //  //m_extension.setDefaultCommand(new ControllerMoveExtension(()-> m_operator.getRawAxis(XboxController.Axis.kRightY.value), m_extension));
//         // m_extension.setDefaultCommand(new MoveExtHome(m_extension, .05));

//         // ledSubsystem.setDefaultCommand(new TeamLights(ledSubsystem));

//         // m_placer.setDefaultCommand(new AnalogPlacer(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_placer));

//         // m_algae.setDefaultCommand(new MoveAlgae(m_algae, () -> m_operator.getRawAxis(XboxController.Axis.kRightY.value)));
       
//         // configureChooser();

//         // configureOperatorBindings();
//         swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));
//         DriverStation.silenceJoystickConnectionWarning(true);
//   }

//     public double driverX() {
//         return driver.getLeftX();
//     }

//     public double driverY() {
//         return driver.getLeftY();
//     }

//     public double driverAngular() {
//         return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
//     }



// }
