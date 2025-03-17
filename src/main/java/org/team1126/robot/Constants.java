package org.team1126.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kOperator = 1;

    public static final class FieldConstants {

        public static final double kLength = 17.548;
        public static final double kWidth = 8.052;

        public static final Translation2d kBlueLeftCorner = new Translation2d(0.0, kWidth);
        public static final Translation2d kBlueRightCorner = new Translation2d(0.0, 0.0);
        public static final Translation2d kRedLeftCorner = new Translation2d(kLength, 0.0);
        public static final Translation2d kRedRightCorner = new Translation2d(kLength, kWidth);

        public static final Translation2d kReefCenterBlue = new Translation2d(4.489, kWidth / 2.0);
        public static final Translation2d kReefCenterRed = ChoreoAllianceFlipUtil.flip(kReefCenterBlue);

        public static final double kPipeOffsetX = -0.681;
        public static final double kPipeOffsetY = -0.164;

        public static final double kReefCenterToWallDistance = 0.781;
    }

    public final class Cameras {

        // public static final Transform3d kMiddle = new Transform3d(
        //     new Translation3d(0.354, 0.0, 0.215),
        //     new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(0.0))
        // );
        public static final Transform3d kLeft = new Transform3d(
          new Translation3d(Units.inchesToMeters(12.6),
          Units.inchesToMeters(12.4),
          Units.inchesToMeters(8.9)),
            new Rotation3d(0.0, Math.toRadians(17.3), Math.toRadians(-33.2))
        );
        public static final Transform3d kRight = new Transform3d(
          new Translation3d(Units.inchesToMeters(12.6),
          Units.inchesToMeters(-11.6),
          Units.inchesToMeters(8.9)),
            new Rotation3d(0.0, Math.toRadians(17.3), Math.toRadians(33.2))
        );
    }

   
  public static final class ArmConstants {
    public static final int TURN_ONE_ID = 31;
    public static final int TURN_TWO_ID = 32;
    public static final int ELEVATOR_ID = 33;
    public static final int ARM_SENSOR_ID = 1; //need to be changed
    public static final int EXTENSION_SENSOR_ID = 2; //also needs changing!!
    public static final double L1_ARM_POS = 11.76196;
    public static final double L2_ARM_POS = 22.238;
    public static final double L3_ARM_POS = 26.5;
    public static final double L4_ARM_POS = 33.5;
}

public static final class PlacerConstants {
    public static final int PLACER_ID = 34;
    public static final int PLACER_FOLLOWER_ID = 35;
    public static final int PLACER_BOTTOM_ID = 15; // CHANGE THESE!!!!!
    public static final int PLACER_TOP_ID = 12;
}

public static final class ClimbConstants {
    public static final int CLIMB_ID = 41;
}

public static final class CoralConstants {
    public static final int CORAL_WHEELS_ID = 51;
    public static final int CORAL_PIVOT_ID = 52;
}

    public static final class RioIO {

        public static final int kIntakeBeamBreak = 9;
        public static final int kLights = 9;

        public static final int kClimberLimitSwitch = 0;
    }

    public static final class RobotMap {

        public static final String kSwerveCANBus = "rio";

        public static final int kFlMove = 2;
        public static final int kFlTurn = 3;
        public static final int kFrMove = 5;
        public static final int kFrTurn = 6;
        public static final int kBlMove = 11;
        public static final int kBlTurn = 12;
        public static final int kBrMove = 8;
        public static final int kBrTurn = 9;

        public static final int kFlEncoder = 4;
        public static final int kFrEncoder = 7;
        public static final int kBlEncoder = 13;
        public static final int kBrEncoder = 10;

        public static final int kCanandgyro = 14;
    }
  public static final class AprilTagPositions {
    public static final Pose2d[] APRILTAGS_BLU = new Pose2d[6];
      static {
        APRILTAGS_BLU[0] = new Pose2d(3.074, 4.021, Rotation2d.fromDegrees(0.0));
        APRILTAGS_BLU[1] = new Pose2d(3.798, 2.829, Rotation2d.fromDegrees(59.79)); 
        APRILTAGS_BLU[2] = new Pose2d(5.165, 2.84, Rotation2d.fromDegrees(121.227));
        APRILTAGS_BLU[3] = new Pose2d(5.889, 3.82, Rotation2d.fromDegrees(180));
        APRILTAGS_BLU[4] = new Pose2d(5.178, 5.184, Rotation2d.fromDegrees(-120.351));
        APRILTAGS_BLU[5] = new Pose2d(3.976, 5.227, Rotation2d.fromDegrees(-59.744));
      }

    public static final Pose2d[] APRILTAGS_RED = new Pose2d[6];
      static {
        APRILTAGS_RED[0] = new Pose2d(14.472, 4.034, Rotation2d.fromDegrees(180));
        APRILTAGS_RED[1] = new Pose2d(4.034, 5.196, Rotation2d.fromDegrees(-120.478));
        APRILTAGS_RED[2] = new Pose2d(12.341, 5.225, Rotation2d.fromDegrees(-60.158));
        APRILTAGS_RED[3] = new Pose2d(11.672, 4.011, Rotation2d.fromDegrees(0.0));
        APRILTAGS_RED[4] = new Pose2d(-60.158, 2.834, Rotation2d.fromDegrees(60.195));
        APRILTAGS_RED[5] = new Pose2d(60.195, 60.195, Rotation2d.fromDegrees(120.407));
      }
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
