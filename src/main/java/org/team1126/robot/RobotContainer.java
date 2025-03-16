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

//     public static final ArmSubsystem m_arm = new ArmSubsystem();
    
//     public static final ExtensionSubsystem m_extension = new ExtensionSubsystem();
    
//     // public static final AlgaeAcquisition m_algae = new AlgaeAcquisition();

//     public static final ClimbSubsystem m_climb = new ClimbSubsystem();

//     public static final PlacerSubsystem m_placer = new PlacerSubsystem();

//     public static CommandXboxController m_driver = new CommandXboxController(0);
//     public static CommandXboxController m_operator = new CommandXboxController(1);

//     public static final LEDs ledSubsystem = new LEDs(0, 200); //PORT IS PWM!!!

//     public static HttpCamera colorCamera;

//     final static SendableChooser<Command> m_chooser = new SendableChooser<>();
    
//   // The robot's subsystems and commands are defined here...
//   final static Swerve swerve ;// = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
//                                                                                 "swerve/neo"));

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


//     NamedCommands.registerCommand("test", Commands.print("I EXIST"));

//             configurePathPlanner();

//             colorCamera = new HttpCamera("Color Camera", "http://10.11.26.11");

//         // human control for climb and algae

//         // m_climb.setDefaultCommand(new ClimbMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftX.value), m_climb));
//         m_arm.setDefaultCommand(new ControllerMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_arm));
//          //m_extension.setDefaultCommand(new ControllerMoveExtension(()-> m_operator.getRawAxis(XboxController.Axis.kRightY.value), m_extension));
//         m_extension.setDefaultCommand(new MoveExtHome(m_extension, .05));

//         ledSubsystem.setDefaultCommand(new TeamLights(ledSubsystem));

//         // m_placer.setDefaultCommand(new AnalogPlacer(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_placer));

//         // m_algae.setDefaultCommand(new MoveAlgae(m_algae, () -> m_operator.getRawAxis(XboxController.Axis.kRightY.value)));
       
//         configureChooser();

//         configureDriverBindings();
//         configureOperatorBindings();
//         DriverStation.silenceJoystickConnectionWarning(true);
//   }

//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be created via the
//    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
//    * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
//    * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
//    * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
//    */
//   private void configureDriverBindings()
//   {
//     //Command driveFieldOrientedDirectAngle      = m_swerve.driveFieldOriented(driveDirectAngle);
//     Command driveFieldOrientedAnglularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);
//     // Command driveRobotOrientedAngularVelocity  = m_swerve.driveFieldOriented(driveRobotOriented);
//     // Command driveSetpointGen = m_swerve.driveWithSetpointGeneratorFieldRelative(
//     //    driveDirectAngle);
//     Command driveFieldOrientedDirectAngleKeyboard      = m_swerve.driveFieldOriented(driveDirectAngleKeyboard);
//     // Command driveFieldOrientedAnglularVelocityKeyboard = m_swerve.driveFieldOriented(driveAngularVelocityKeyboard);
//     // Command driveSetpointGenKeyboard = m_swerve.driveWithSetpointGeneratorFieldRelative(
//     //     driveDirectAngleKeyboard);

//     if (RobotBase.isSimulation())
//     {
//       m_swerve.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
//     } else
//     {
//       m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
//     }

//     if (Robot.isSimulation())
//     {
//       Pose2d target = new Pose2d(new Translation2d(1, 4),
//                                  Rotation2d.fromDegrees(90));
//       //m_swerve.getSwerveDrive().field.getObject("targetPose").setPose(target);
//       driveDirectAngleKeyboard.driveToPose(() -> target,
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(5, 2)),
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(Units.degreesToRadians(360),
//                                                                                      Units.degreesToRadians(180))
//                                            ));
//       m_driver.start().onTrue(Commands.runOnce(() -> m_swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
//       m_driver.button(1).whileTrue(m_swerve.sysIdDriveMotorCommand());
//       m_driver.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
//                                                      () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

// //      m_driver.b().whileTrue(
// //          m_swerve.driveToPose(
// //              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
// //                              );

//     }
//     if (DriverStation.isTest())
//     {
//       m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

//       m_driver.x().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());
//       m_driver.y().whileTrue(m_swerve.driveToDistanceCommand(1.0, 0.2));
//       m_driver.start().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
//       m_driver.back().whileTrue(m_swerve.centerModulesCommand());
//       m_driver.leftBumper().onTrue(Commands.none());
//       m_driver.rightBumper().onTrue(Commands.none());
//     } else
//     {
//        m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
//             // m_driver.leftTrigger().onChange(new InstantCommand(() -> m_swerve.zeroGyroWithAlliance()));
//             m_driver.a().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
//             m_driver.y().whileTrue(new ClimbMoveToPos(m_climb, 0));
//             m_driver.x().whileTrue(new ClimbMoveToPos(m_climb, 125));
//             m_driver.b().whileTrue(new ClimbMoveToPos(m_climb, -112.55));
//             // m_driver.leftBumper().whileTrue(new DriveToClosestLeftBranchPoseCommand(m_swerve));  
//             // m_driver.leftBumper().whileTrue(m_swerve.driveToPose(m_swerve.getClosestLeftBranchPose()));
//             m_driver.leftBumper().whileTrue(m_swerve.driveToPose(AprilTagPositions.APRILTAGS_BLU[0]));
//             m_driver.rightBumper().whileTrue(m_swerve.driveToPose(m_swerve.getClosestRightBranchPose()));


//       // m_driver.a().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
//       // m_driver.x().onTrue(Commands.runOnce(m_swerve::addFakeVisionReading));
//       // m_driver.start().whileTrue(Commands.none());
//       // m_driver.back().whileTrue(Commands.none());
//       // // m_driver.leftBumper().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());
//       // m_driver.rightBumper().onTrue(Commands.none());
//       // m_driver.leftBumper().whileTrue(m_swerve.driveToPose(new Pose2d(3.074, 4.021, Rotation2d.fromDegrees(0.0))));


//     }

//   }

//       public void configureOperatorBindings() {   

//         m_operator.povDown().whileTrue(new MoveArmToAngle(m_arm, 0).alongWith(new MoveExtensionToPos(m_extension, m_arm, 0.01))); //arm home
//         m_operator.povUp().whileTrue(new MoveArmToAngle(m_arm, 18.442849922180176).alongWith(new MoveExtensionToPos(m_extension, m_arm, .01))
//                   .alongWith(new IngestCoral(m_placer, -.5)).andThen(new PositionCoral(m_placer)));                                                    //arm to coral station

//         m_operator.a().whileTrue(new MoveArmToAngle(m_arm, ArmConstants.L1_ARM_POS).alongWith(new MoveExtensionToPos(m_extension, m_arm, 0.013659)).alongWith(new ReefLights(ledSubsystem, true, 1))); //arm l1
//         m_operator.x().whileTrue(new MoveArmToAngle(m_arm, ArmConstants.L2_ARM_POS).alongWith(new MoveExtensionToPos(m_extension, m_arm,-0.0831989)).alongWith(new ReefLights(ledSubsystem, true, 2))); //arm l2
//         m_operator.b().whileTrue(new MoveArmToAngle(m_arm,  ArmConstants.L3_ARM_POS).alongWith(new MoveExtensionToPos(m_extension, m_arm, -0.25)).alongWith(new ReefLights(ledSubsystem, true, 3))); //arm l3
//         m_operator.y().whileTrue(new MoveArmToAngle(m_arm, ArmConstants.L4_ARM_POS).alongWith(new MoveExtensionToPos(m_extension, m_arm, -0.55)).alongWith(new ReefLights(ledSubsystem, true, 4))); //arm l4

//         m_operator.rightTrigger(0.1).whileTrue(new AnalogPlacer(() -> m_operator.getRawAxis(XboxController.Axis.kRightTrigger.value), m_placer,false));
//         m_operator.leftTrigger(0.1).whileTrue(new AnalogPlacer(() -> m_operator.getRawAxis(XboxController.Axis.kLeftTrigger.value), m_placer,true));
//         // m_operator.povRight().whileTrue(new AlgaeMoveToPosition(m_algae, 40));
//         // m_operator.leftBumper().whileTrue(new AlgaeMoveToHome(m_algae));
//         // m_operator.rightBumper().whileTrue(new SpitAlgae(m_algae));
//     }

//       public void configureChooser() {
//         // autos using pathplanner
//         m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
//         // m_chooser.addOption("3 CORAL AUTO", new PathPlannerAuto("3CoralAuto"));
//         // m_chooser.addOption("Test", new PathPlannerAuto("Startpos1 l4 coral"));

//           m_chooser.addOption("Dump L1 R BLUE", new PathPlannerAuto("DumpL1 R BLU"));
//           m_chooser.addOption("Dump L1 M BLUE", new PathPlannerAuto("DumpL1 M BLU"));
//           m_chooser.addOption("Dump L1 L BLUE", new PathPlannerAuto("DumpL1 L BLU"));

//           m_chooser.addOption("Dump L1 L RED", new PathPlannerAuto("DumpL1 L RED"));
//           m_chooser.addOption("Dump L1 R RED", new PathPlannerAuto("DumpL1 R RED"));
//           m_chooser.addOption("Dump L1 M RED", new PathPlannerAuto("DumpL1 M RED"));

//           m_chooser.addOption("Coral L4 M RED", new PathPlannerAuto("Score L4 M"));
//           m_chooser.addOption("Coral L4 M BLU", new PathPlannerAuto("Score L4 M BLU"));

//         m_chooser.addOption("MoveForward RED",new PathPlannerAuto("MoveForward RED"));
//         m_chooser.addOption("MoveForward BLUE",new PathPlannerAuto("MoveForward BLU"));

//       }

//    /* REGISTER PATHPLANNER COMMANDS HERE */
//     public void configurePathPlanner() {
        
//         NamedCommands.registerCommand("Wait", new WaitCommand(1));

//         NamedCommands.registerCommand("MoveArmToHome",new MoveArmToAngle(m_arm, -.01).withTimeout(1));
//         NamedCommands.registerCommand("MoveExtensionToHome", new MoveExtHome(m_extension, -0.01).withTimeout(1));

//         NamedCommands.registerCommand("MoveArmToCoral", new MoveArmToAngle(m_arm, 17.642849922180176).withTimeout(1));
//         NamedCommands.registerCommand("MoveExtensionToCoral", new MoveExtensionToPos(m_extension, m_arm, .01).withTimeout(1));
//         NamedCommands.registerCommand("AcquireCoral", new IngestCoral(m_placer,-.35).andThen(new PositionCoral(m_placer)));

//         NamedCommands.registerCommand("MoveArmToL1",new MoveArmToAngle(m_arm, 11.76196).withTimeout(1));
//         NamedCommands.registerCommand("MoveArmToL2", new MoveArmToAngle(m_arm, 22.238).withTimeout(1));
//         NamedCommands.registerCommand("MoveArmToL3", new MoveArmToAngle(m_arm,  26.5).withTimeout(1));
//         NamedCommands.registerCommand("MoveArmToL4", new MoveArmToAngle(m_arm, 33.5).withTimeout(1));

//         NamedCommands.registerCommand("MoveExtensionToL1", new MoveExtensionToPos(m_extension,m_arm, 0.013659).withTimeout(1));
//         NamedCommands.registerCommand("MoveExtensionToL2", new MoveExtensionToPos(m_extension, m_arm,-0.0831989).withTimeout(1));
//         NamedCommands.registerCommand("MoveExtensionToL3", new MoveExtensionToPos(m_extension, m_arm,-0.25).withTimeout(1));
//         NamedCommands.registerCommand("MoveExtensionToL4", new MoveExtensionToPos(m_extension, m_arm, -0.55).withTimeout(2));


//         NamedCommands.registerCommand("SpinPlacerOut", new PlaceCoral(m_placer, 0.5).withTimeout(1));
//         NamedCommands.registerCommand("SlowPlacer", new PlaceCoral(m_placer, 0.3).withTimeout(1));
//         NamedCommands.registerCommand("SpinPlacerIn",new AcquireCoral(m_placer).withTimeout(1));

//         NamedCommands.registerCommand("IngestCoral", new IngestCoral(m_placer,-.15));
//         NamedCommands.registerCommand("PositionCoral", new PositionCoral(m_placer));

//         // NamedCommands.registerCommand("MovetoPos", new InstantCommand(() -> m_swerve.moveToPosition) );  
//          }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand()
//   {
//     // An example command will be run in autonomous
//     //return m_swerve.getAutonomousCommand("New Auto");

//     return m_chooser.getSelected();
//   }

//   public void setMotorBrake(boolean brake)
//   {
//     m_swerve.setMotorBrake(brake);
//   }
  
// }
