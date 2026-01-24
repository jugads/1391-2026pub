// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.WantedState;

/** Add your docs here. */
public class RobotCore extends SubsystemBase {

  ShooterSubsystem shooter;
  GroundIntakeSubsystem groundIntake;
  HopperSubsystem hopper;
  CommandSwerveDrivetrain drivetrain;
  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
  private boolean readyToShoot = false;

  public RobotCore(
    ShooterSubsystem shooter,
    GroundIntakeSubsystem groundIntake,
    HopperSubsystem hopper,
    CommandSwerveDrivetrain drivetrain
  ) {
    this.shooter = shooter;
    this.groundIntake = groundIntake;
    this.hopper = hopper;
    this.drivetrain = drivetrain;
  }

  public enum WantedSuperState {
    IDLE,
    REV,
    SHOOT,
    INTAKE,
    REVERSE_INTAKE,
    LOAD,
  }

  public enum CurrentSuperState {
    IDLING,
    REVVING,
    SHOOTING,
    INTAKING,
    REVERSING_INTAKE,
    LOADING,
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
  }

  public void handleStateTransitions() {
    switch (wantedSuperState) {
      case IDLE:
        currentSuperState = CurrentSuperState.IDLING;
        break;
      case REV:
        if (readyToShoot) {
          currentSuperState = CurrentSuperState.LOADING;
        } else {
          currentSuperState = CurrentSuperState.REVVING;
        }
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
    }
  }

  public void applyStateBehavior() {
    switch (currentSuperState) {
      case IDLING:
        groundIntake.stop();
        hopper.stop();
        shooter.stop();
        break;
      case REVVING:
        //ADD DRIVETRAIN METHODS TO HUB
        shooter.shoot(shooter.calculateShooterSpeed(0.0));
        break;
      case LOADING:
        shooter.setWantedState(ShooterSubsystem.WantedState.LOAD);
        hopper.feed(1.);
        break;
      case SHOOTING:
        shooter.setWantedState(ShooterSubsystem.WantedState.SHOOT_AT_HUB);
        hopper.feed(1.);
        break;
      case INTAKING:
        groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.INTAKE);
        break;
      case REVERSING_INTAKE:
        groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.REVERSE);
        break;
    }
  }
}
