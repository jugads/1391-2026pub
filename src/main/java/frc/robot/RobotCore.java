// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.GroundIntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.util.Limelight;
import java.lang.constant.DirectMethodHandleDesc;

/** Add your docs here. */
public class RobotCore extends SubsystemBase {

  ShooterSubsystem shooter;
  GroundIntakeSubsystem groundIntake;
  HopperSubsystem hopper;
  CommandSwerveDrivetrain drivetrain;
  LEDs leds;
  Limelight tagLimelight;
  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
  private boolean readyToShoot = false;
  private boolean intakeOverride = false;
  private boolean shootingAtHub = true;
  private double shooterCalculatedSpeed = 0.0;
  private boolean hasCalculatedShooterSpeed = false;
  private boolean toggleRevving = true;
  private boolean wiggle = false;
  double timeOfStartedShooting;
  private boolean hasCalculatedTimeOfShooting = false;

  public RobotCore(
    ShooterSubsystem shooter,
    GroundIntakeSubsystem groundIntake,
    HopperSubsystem hopper,
    CommandSwerveDrivetrain drivetrain,
    LEDs leds,
    Limelight tagLimelight
  ) {
    this.shooter = shooter;
    this.groundIntake = groundIntake;
    this.hopper = hopper;
    this.drivetrain = drivetrain;
    this.leds = leds;
    this.tagLimelight = tagLimelight;
    SmartDashboard.putNumber("shooter speed", 0);
  }

  public enum WantedSuperState {
    IDLE,
    SHOOT,
    SHOOT_FROM_DISTANCE,
    INTAKE,
    REVERSE_INTAKE,
    LOAD,
    HOME,
    REV_AUTO,
    SHOOT_WHILE_MOVING,
  }

  public enum CurrentSuperState {
    IDLING,
    SHOOTING,
    SHOOTING_FROM_DISTANCE,
    INTAKING,
    REVERSING_INTAKE,
    LOADING,
    HOMED,
    REVVING_AUTO,
    SHOOTING_WHILE_MOVING,
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
    SmartDashboard.putBoolean("Superstructure/Intake Override", intakeOverride);
    SmartDashboard.putNumber(
      "Superstructure/Shooter Speed Setpoint",
      shooterCalculatedSpeed
    );
    SmartDashboard.putBoolean(
      "Superstructure/Is Shooting At Hub",
      shootingAtHub
    );
    SmartDashboard.putBoolean(
      "Has Calculated time",
      hasCalculatedTimeOfShooting
    );
    SmartDashboard.putString(
      "Superstructure/Current State",
      currentSuperState.toString()
    );
    SmartDashboard.putBoolean("wiggle", wiggle);
  }

  public void handleStateTransitions() {
    switch (wantedSuperState) {
      case IDLE:
        currentSuperState = CurrentSuperState.IDLING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case SHOOT_FROM_DISTANCE:
        currentSuperState = CurrentSuperState.SHOOTING_FROM_DISTANCE;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case REVERSE_INTAKE:
        currentSuperState = CurrentSuperState.REVERSING_INTAKE;
        break;
      case HOME:
        currentSuperState = CurrentSuperState.HOMED;
        intakeOverride = false;
        hasCalculatedTimeOfShooting = false;
        break;
      case REV_AUTO:
        currentSuperState = CurrentSuperState.REVVING_AUTO;
        break;
      case SHOOT_WHILE_MOVING:
        if (drivetrain.getVelocityVector() < 0.25) currentSuperState =
          CurrentSuperState.SHOOTING_FROM_DISTANCE;
        else currentSuperState = CurrentSuperState.SHOOTING_WHILE_MOVING;
        break;
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
        shooter.shoot(SmartDashboard.getNumber("shooter speed", 5000));
        if (shooter.isUpToSpeed()) {
          if (!intakeOverride) {
            shooter.shootAtHub();
          } else {
            shooter.shootAtHubRegular();
          }
          hopper.setWantedState(HopperSubsystem.WantedState.FEED);
        }
        break;
      case SHOOTING_FROM_DISTANCE:
        if (!hasCalculatedShooterSpeed) {
          if (tagLimelight.isSeeingValidTarget()) {
            drivetrain.resetGlobalPose(
              tagLimelight.getLimelightPoseEstimateData().pose
            );
          }
          shooterCalculatedSpeed = shooter.calculateShooterSpeed(
            drivetrain.getDistanceFromHub()
          );
          hasCalculatedShooterSpeed = true;
        }
        shooter.shoot(shooterCalculatedSpeed + 250);
        if (shooter.isUpToSpeed()) {
          if (!hasCalculatedTimeOfShooting) {
            hasCalculatedTimeOfShooting = true;
            timeOfStartedShooting = Timer.getFPGATimestamp();
          }
          if ((Timer.getFPGATimestamp() - timeOfStartedShooting) > 1.5) {
            intakeOverride = true;
          }
          hopper.setWantedState(HopperSubsystem.WantedState.FEED);
          shooter.feedAndShoot(shooterCalculatedSpeed + 30);
        }
        break;
      case SHOOTING_WHILE_MOVING:
        if (!hasCalculatedShooterSpeed) {
          if (tagLimelight.isSeeingValidTarget()) {
            drivetrain.resetGlobalPose(
              tagLimelight.getLimelightPoseEstimateData().pose
            );
          }
          hasCalculatedShooterSpeed = true;
        }
        double setpoint =
          shooter.calculateShooterSpeed(drivetrain.getDistanceFromHub()) +
          drivetrain.getRateOfChangeOfDistanceFromHubMetersPerSecond() * 200;
        shooter.feedAndShoot(setpoint);
        hopper.feed(0.5);
        SmartDashboard.putNumber("Setpoint", setpoint);
        break;
      case REVVING_AUTO:
        shooter.setWantedState(ShooterSubsystem.WantedState.REV_TO_SPEED);
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        break;
      case INTAKING:
        shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        break;
      case REVERSING_INTAKE:
        hopper.setWantedState(HopperSubsystem.WantedState.REVERSE);
        shooter.stop();
        break;
      case HOMED:
        hasCalculatedShooterSpeed = false;
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        if (
          drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
          toggleRevving
        ) {
          shooter.setWantedState(ShooterSubsystem.WantedState.WARM_UP);
        } else {
          shooter.stop();
        }
        break;
    }
    if (
      (intakeOverride || wiggle) &&
      currentSuperState != CurrentSuperState.INTAKING
    ) {
      if (!wiggle) {
        groundIntake.setWantedState(
          GroundIntakeSubsystem.WantedState.HOLD_AT_ZERO
        );
      } else {
        if (
          groundIntake.getWantedState() !=
          GroundIntakeSubsystem.WantedState.HOLD_AT_ZERO
        ) {
          groundIntake.wiggle();
        }
      }
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
        case REVVING_AUTO:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.HOLD_AT_DEFAULT
          );
          break;
        case SHOOTING_FROM_DISTANCE:
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

  public void setWantedSuperState(WantedSuperState state) {
    this.wantedSuperState = state;
  }

  public boolean canDriveUnderTrenchSafely() {
    return groundIntake.getIntakePivotAngle() > kINTAKE_MAX_ANGLE_UNDER_TRENCH;
  }

  public Command setIntakeOverrideCommand(boolean override) {
    return new InstantCommand(() -> this.intakeOverride = override);
  }

  public Command toggleAutoRev() {
    return new InstantCommand(() -> this.toggleRevving = (!toggleRevving));
  }

  public Command shootFuel(boolean atHub) {
    hasCalculatedShooterSpeed = false;
    if (atHub) return new InstantCommand(() ->
      wantedSuperState = WantedSuperState.SHOOT
    );
    else return new InstantCommand(() ->
      wantedSuperState = WantedSuperState.SHOOT_FROM_DISTANCE
    );
  }

  private LEDs.WantedState computeLedState() {
    if (!DriverStation.isEnabled()) return LEDs.WantedState.DISABLED;
    if (
      wantedSuperState == WantedSuperState.SHOOT ||
      wantedSuperState == WantedSuperState.SHOOT_FROM_DISTANCE
    ) {
      return shooter.isUpToSpeed()
        ? LEDs.WantedState.SHOOT_READY
        : LEDs.WantedState.SHOOT_SPINUP;
    }
    if (
      currentSuperState == CurrentSuperState.INTAKING
    ) return LEDs.WantedState.INTAKE;
    if (
      drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
      drivetrain.getDistanceFromHub() < 2.6 &&
      drivetrain.getDistanceFromHub() > 1.95
    ) return LEDs.WantedState.GREEN;
    if (
      drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
      drivetrain.getDistanceFromHub() < 1.95
    ) return LEDs.WantedState.IDEAL;
    return LEDs.WantedState.IDLE;
  }

  public Command toggleIntake() {
    return new InstantCommand(() -> {
      if (wantedSuperState == WantedSuperState.INTAKE) {
        wantedSuperState = WantedSuperState.HOME;
      } else {
        wantedSuperState = WantedSuperState.INTAKE;
      }
    });
  }

  public boolean cameraSeesTag() {
    return tagLimelight.getLimelightPoseEstimateData().tagCount > 0;
  }

  public Pose2d getLimelightPose() {
    return tagLimelight.getLimelightPoseEstimateData().pose;
  }

  public Command shootWhileMoving() {
    return new InstantCommand(() ->
      this.wantedSuperState = WantedSuperState.SHOOT_WHILE_MOVING
    );
  }

  public Command toggleWiggle() {
    return new InstantCommand(() -> this.wiggle = !this.wiggle);
  }
}
