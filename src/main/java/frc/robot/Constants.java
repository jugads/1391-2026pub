// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Collection;
import java.util.List;

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
    public static final double kINTAKING_POSITION_SETPOINT = 5.35;
    public static final double kIDLED_POSITION_SETPOINT = 5.35;
    public static final double kZERO_SETPOINT = 0.0;
    public static final double kINTAKE_MAX_ANGLE_UNDER_TRENCH = 5.0;
    public static final double kENCODER_OFFSET = 0.313;
    public static final int kENCODER_PORT = 9;
  }

  public class ShooterConstants {

    public static final double kSHOOTER_SPEED_AT_HUB = 3500.0;
    public static final double kSHOOTER_SPEED_TOLERANCE = 200.;
    public static final double kREVERSING_SPEED = 0.0;
    public static final InterpolatingDoubleTreeMap kSHOOTER_SPEEDS =
      new InterpolatingDoubleTreeMap();
    public static final Double[] kSHOOTER_ENTRY_0 = new Double[] { 1.6, 3525. };
    public static final Double[] kSHOOTER_ENTRY_1 = new Double[] { 1.7, 3625. };
    public static final Double[] kSHOOTER_ENTRY_2 = new Double[] { 1.8, 3700. };
    public static final Double[] kSHOOTER_ENTRY_3 = new Double[] { 1.9, 3850. };
    public static final Double[] kSHOOTER_ENTRY_4 = new Double[] { 2.0, 4125. };
    public static final Double[] kSHOOTER_ENTRY_5 = new Double[] { 2.1, 4250. };
    public static final Double[] kSHOOTER_ENTRY_6 = new Double[] {
      2.2,
      4375.0,
    };
    public static final Double[] kSHOOTER_ENTRY_7 = new Double[] {
      2.3,
      4600.0,
    };
    public static final Double[] kSHOOTER_ENTRY_8 = new Double[] {
      2.4,
      4700.0,
    };
    public static final Double[] kSHOOTER_ENTRY_9 = new Double[] {
      2.5,
      4900.0,
    };
    public static final Double[] kSHOOTER_ENTRY_10 = new Double[] {
      2.6,
      5100.0,
    };
    public static final Double[] kSHOOTER_ENTRY_11 = new Double[] {
      2.7,
      5250.0,
    };
    public static final Double[] kSHOOTER_ENTRY_12 = new Double[] {
      2.8,
      5400.0,
    };
    public static final Double[] kSHOOTER_ENTRY_13 = new Double[] {
      2.9,
      5550.0,
    };
    public static final Double[] kSHOOTER_ENTRY_14 = new Double[] {
      3.0,
      5700.0,
    };
    public static final Double[] kSHOOTER_ENTRY_15 = new Double[] {
      3.1,
      5800.0,
    };

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

    public static final List<Pose2d> kTRENCHES = List.of(
      new Pose2d(3.54, 0.67, new Rotation2d(0.0)), // RIGHT BLUE
      new Pose2d(12.99, 0.67, new Rotation2d(Math.PI)), // LEFT RED
      new Pose2d(12.99, 7.49, new Rotation2d(Math.PI)), // RIGHT RED
      new Pose2d(3.54, 7.49, new Rotation2d(0.0)) // LEFT BLUE
    );

    public class TidalLockConstants {

      public static final double kP = 0.15;
      public static final double kI = 0.1;
      public static final double kD = 0.0;
      public static final double kVELOCITY_MULTIPLIER = 0.1;
    }
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
