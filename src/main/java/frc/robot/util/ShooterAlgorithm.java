// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.ShooterConstants.*;

/** Add your docs here. */
public class ShooterAlgorithm {

  public ShooterAlgorithm() {
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_000[0], kSHOOTER_ENTRY_000[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_001[0], kSHOOTER_ENTRY_001[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_0002[0], kSHOOTER_ENTRY_0002[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_00[0], kSHOOTER_ENTRY_00[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_01[0], kSHOOTER_ENTRY_01[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_02[0], kSHOOTER_ENTRY_02[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_03[0], kSHOOTER_ENTRY_03[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_04[0], kSHOOTER_ENTRY_04[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_05[0], kSHOOTER_ENTRY_05[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_06[0], kSHOOTER_ENTRY_06[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_07[0], kSHOOTER_ENTRY_07[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_08[0], kSHOOTER_ENTRY_08[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_09[0], kSHOOTER_ENTRY_09[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_10[0], kSHOOTER_ENTRY_10[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_11[0], kSHOOTER_ENTRY_11[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_12[0], kSHOOTER_ENTRY_12[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_13[0], kSHOOTER_ENTRY_13[1]);

    k_SHOOTER_SPEED_MAP_DCMP.put(kSHOOTER_ENTRY_HUB_DCMP[0], kSHOOTER_ENTRY_HUB_DCMP[1]);
    k_SHOOTER_SPEED_MAP_DCMP.put(k_SHOOTER_ENTRY_FAR_DCMP[0], k_SHOOTER_ENTRY_FAR_DCMP[1]);

  }
  /**
   * Calculate the shooter speed using the regular interpolation map.
   *
   * <p>If the distance is less than 1.5, the shooter speed at the hub is
   * returned. Otherwise, the shooter speed is interpolated using the map.
   *
   * @param distance the distance from the hub to the target, in meters
   * @return the shooter speed, in RPM
   */
  public double calculateShooterSpeedRegular(double distance) {
    if (distance < 1.5) {
      return kSHOOTER_SPEED_AT_HUB;
    }
    else {
      return kSHOOTER_SPEEDS.get(distance);
    }
  }

  /**
   * Calculate the shooter speed using the DCMP interpolation map.
   *
   * <p>If the distance is less than 1.5, the shooter speed at the hub is
   * returned. Otherwise, the shooter speed is interpolated using the map.
   *
   * @param distance the distance from the hub to the target, in meters
   * @return the shooter speed, in RPM
   */
  public double calculateShooterSpeedDCMP(double distance) {
    if (distance < 1.5) {
      return kSHOOTER_SPEED_AT_HUB;
    }
    else {
      return k_SHOOTER_SPEED_MAP_DCMP.get(distance);
    }
  }

  public double calculateHoodAngle(double distance) {
    return 0.0;
  }
}
