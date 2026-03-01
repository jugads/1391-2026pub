// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.GroundIntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.kSHOOTER_SPEED_AT_HUB;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem.WantedState;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/** Add your docs here. */
public class RobotCore extends SubsystemBase {

  ShooterSubsystem shooter;
  GroundIntakeSubsystem groundIntake;
  HopperSubsystem hopper;
  CommandSwerveDrivetrain drivetrain;
  LEDs leds;
  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
  private boolean readyToShoot = false;
  private boolean intakeOverride = false;
  private boolean shootingAtHub = true;

  public RobotCore(
    ShooterSubsystem shooter,
    GroundIntakeSubsystem groundIntake,
    HopperSubsystem hopper,
    CommandSwerveDrivetrain drivetrain,
    LEDs leds
  ) {
    this.shooter = shooter;
    this.groundIntake = groundIntake;
    this.hopper = hopper;
    this.drivetrain = drivetrain;
    this.leds = leds;
  }

  public enum WantedSuperState {
    IDLE,
    SHOOT,
    INTAKE,
    REVERSE_INTAKE,
    LOAD,
    HOME,
  }

  public enum CurrentSuperState {
    IDLING,
    SHOOTING,
    INTAKING,
    REVERSING_INTAKE,
    LOADING,
    HOMED,
  }

  @Override
  public void periodic() {
    handleStateTransitions();
    applyStateBehavior();
    if (shooter.isUpToSpeed()) {
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }
    SmartDashboard.putBoolean("Superstructure/Ready to Shoot", readyToShoot);
    SmartDashboard.putBoolean("Intake Override", intakeOverride);
  }

  public void handleStateTransitions() {
    switch (wantedSuperState) {
      case IDLE:
        currentSuperState = CurrentSuperState.IDLING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case REVERSE_INTAKE:
        currentSuperState = CurrentSuperState.REVERSING_INTAKE;
        break;
      case HOME:
        currentSuperState = CurrentSuperState.HOMED;
      default:
        currentSuperState = CurrentSuperState.HOMED;
    }
  }

  public void applyStateBehavior() {
    switch (currentSuperState) {
      case IDLING:
        hopper.stop();
        shooter.stop();
        break;
      case SHOOTING:
        shooter.shoot(kSHOOTER_SPEED_AT_HUB);

        if (shootingAtHub) {
          if (shooter.isUpToSpeed()) {
            shooter.shootAtHub();
            hopper.setWantedState(HopperSubsystem.WantedState.FEED);
          }
        } else {
          shooter.shoot(
            shooter.calculateShooterSpeed(drivetrain.getDistanceFromHub())
          );
        }
        break;
      case INTAKING:
        shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        break;
      case REVERSING_INTAKE:
        hopper.setWantedState(HopperSubsystem.WantedState.REVERSE);
        break;
      case HOMED:
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
        break;
    }
    if (intakeOverride && currentSuperState != CurrentSuperState.INTAKING) {
      groundIntake.setWantedState(
        GroundIntakeSubsystem.WantedState.HOLD_AT_ZERO
      );
    } else {
      switch (currentSuperState) {
        case INTAKING:
          groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.INTAKE);
          break;
        case REVERSING_INTAKE:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.REVERSE
          );
          break;
        case HOMED:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.HOLD_AT_DEFAULT
          );
          break;
        case SHOOTING:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.HOLD_AT_DEFAULT
          );
          break;
        default:
          groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.IDLE);
      }
    }
    leds.setWantedState(computeLedState());
  }

  public CommandSwerveDrivetrain fetchDrivetrain() {
    return drivetrain;
  }

  public Command setWantedSuperStateCommand(WantedSuperState state) {
    return new InstantCommand(() -> this.wantedSuperState = state);
  }

  public boolean canDriveUnderTrenchSafely() {
    return groundIntake.getIntakePivotAngle() < kINTAKE_MAX_ANGLE_UNDER_TRENCH;
  }

  public Command setIntakeOverrideCommand(boolean override) {
    return new InstantCommand(() -> this.intakeOverride = override);
  }

  public Command shootFuel(boolean atHub) {
    this.shootingAtHub = atHub;
    return new InstantCommand(() -> wantedSuperState = WantedSuperState.SHOOT);
  }

  private LEDs.WantedState computeLedState() {
    if (!DriverStation.isEnabled()) return LEDs.WantedState.DISABLED;
    if (wantedSuperState == WantedSuperState.SHOOT) {
      return shooter.isUpToSpeed()
        ? LEDs.WantedState.SHOOT_READY
        : LEDs.WantedState.SHOOT_SPINUP;
    }
    if (
      currentSuperState == CurrentSuperState.INTAKING
    ) return LEDs.WantedState.INTAKE;
    return LEDs.WantedState.IDLE;
  }
}
