// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class Constants {
  public static final String kCANBUSNAME = "canivore";
  public class GroundIntakeConstants {

    public static final double kS = 0.15;
    public static final double kG = 0.25;
    public static final double kV = 0.23;
    public static final double kP = 3.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public double intakeSpeed = 0.0;
    public double encoderPosition = 0.0;

    public static final double MIN_WHEEL_SPEED = 0;
    public static final double MIN_PIVOT_SPEED = 0;
    public static final double feedingIntakeSpeed = 0.0;
    public static final double kINTAKING_POSITION_SETPOINT = 5.7;
    public static final double kIDLED_POSITION_SETPOINT = 5.7;
    public static final double kZERO_SETPOINT = 0.0;
    public static final double kINTAKE_MAX_ANGLE_UNDER_TRENCH = 0.0;
    public static final double kENCODER_OFFSET = 0.313;
    public static final int kENCODER_PORT = 9;
  }

  public class ShooterConstants {

    public static final double kSHOOTER_SPEED_AT_HUB = 4250.0;
    public static final double kSHOOTER_SPEED_TOLERANCE = 100.;
    public static final double kREVERSING_SPEED = 0.0;
    public static final InterpolatingDoubleTreeMap kSHOOTER_SPEEDS =
      new InterpolatingDoubleTreeMap();
    public static final Double[] kSHOOTER_ENTRY_0 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_1 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_2 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_3 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_4 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_5 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_6 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_7 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_8 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_9 = new Double[] { 0.0, 0.0 };
    public static final Double[] kSHOOTER_ENTRY_10 = new Double[] { 0.0, 0.0 };

    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.11;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
  }

  public class HopperConstants {

    public static final double kBELT_SPEED = 0.0;
  }

  public class VisionConstants {

    public static final int kLED_PORT = 8;
    public static final String kAPRILTAG_LL_NAME = "limelight-intake";
    public static final Translation2d kBLUEHUBPOSE = new Translation2d(
      4.634,
      4.029
    );
    public static final Translation2d kREDHUBPOSE = new Translation2d(
      11.919,
      4.029
    );

    public class TidalLockConstants {

      public static final double kP = 0.15;
      public static final double kI = 0.1;
      public static final double kD = 0.0;
      public static final double kVELOCITY_MULTIPLIER = 0.1;
    }
  }

  public class AutonomousConstants {

    public static final Pose2d kSTARTING_POSE_RIGHT_SIDE = new Pose2d(
      3.633,
      0.415,
      Rotation2d.fromDegrees(0)
    );
  }

  public class MotorIDConstants {

    public static final int kPIVOT_ID = 35;
    public static final int kINTAKE_ID1 = 36;
    public static final int kINTAKE_ID2 = 37;
    public static final int kHOP_ID = 20;
    public static final int kSHOOTER_ID = 51;
    public static final int kLOADER_ID = 50;
  }
}
