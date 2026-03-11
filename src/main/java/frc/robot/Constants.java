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

    public static final double kSHOOTER_SPEED_AT_HUB = 4250.0;
    public static final double kSHOOTER_SPEED_TOLERANCE = 100.;
    public static final double kREVERSING_SPEED = 0.0;
    public static final double kMAX_DISTANCE_FROM_HUB = 2.8;
    public static final double kMIN_DISTANCE_FROM_HUB = 1.51;
    public static final InterpolatingDoubleTreeMap kSHOOTER_SPEEDS =
      new InterpolatingDoubleTreeMap();
    public static final Double[] kSHOOTER_ENTRY_00 = new Double[] {
      1.6,
      4325.,
    };
    public static final Double[] kSHOOTER_ENTRY_01 = new Double[] {
      1.7,
      4425.,
    };
    public static final Double[] kSHOOTER_ENTRY_02 = new Double[] {
      1.8,
      4630.,
    };
    public static final Double[] kSHOOTER_ENTRY_03 = new Double[] {
      1.9,
      4850.,
    };
    public static final Double[] kSHOOTER_ENTRY_04 = new Double[] {
      2.0,
      4875.,
    };
    public static final Double[] kSHOOTER_ENTRY_05 = new Double[] {
      2.1,
      4975.,
    };
    public static final Double[] kSHOOTER_ENTRY_06 = new Double[] {
      2.2,
      5150.0,
    };
    public static final Double[] kSHOOTER_ENTRY_07 = new Double[] {
      2.3,
      5250.0,
    };
    public static final Double[] kSHOOTER_ENTRY_08 = new Double[] {
      2.4,
      5350.0,
    };
    public static final Double[] kSHOOTER_ENTRY_09 = new Double[] {
      2.5,
      5550.0,
    };
    public static final Double[] kSHOOTER_ENTRY_10 = new Double[] {
      2.6,
      5650.0,
    };
    public static final Double[] kSHOOTER_ENTRY_11 = new Double[] {
      2.7,
      5750.0,
    };
    public static final Double[] kSHOOTER_ENTRY_12 = new Double[] {
      2.8,
      5800.0,
    };

    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.105;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
  }

  public class HopperConstants {

    public static final double kBELT_SPEED = 0.0;
  }

  public class VisionConstants {

    public static final int kLED_PORT = 9;
    public static final String kAPRILTAG_LL_NAME = "limelight-intake";
    public static final Translation2d kBLUEHUBPOSE = new Translation2d(
      4.634,
      4.029
    );
    public static final Translation2d kREDHUBPOSE = new Translation2d(
      11.919,
      4.029
    );

    public static final double[] kTRENCH_GOING_OUT_SETPOINTS = new double[] {
      -18,-1,0
    };
    public static final double[] kTRENCH_GOING_IN_SETPOINTS = new double[] {
      0,0,180
    };

    public class TidalLockConstants {

      public static final double kP = 0.08;
      public static final double kI = 0.;
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
    public static final int kSHOOTER_FOLLOWER_ID = 52;
    public static final int kLOADER_FOLLOWER_ID = 53;
  }
}
