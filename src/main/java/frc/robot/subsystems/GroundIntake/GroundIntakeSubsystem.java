package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;

import com.ctre.phoenix6.hardware.*;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {

  private final GroundIntakeIO io;
  private final GroundIntakeIO.GroundIntakeIOInputs inputs =
    new GroundIntakeIO.GroundIntakeIOInputs();
  double setpoint;

  public enum WantedState {
    IDLE,
    INTAKE,
    HOLD_AT_DEFAULT,
    REVERSE,
  }

  private enum SystemState {
    IDLED,
    INTAKING,
    HOLDING_AT_DEFAULT,
    REVERSING,
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLED;

  public GroundIntakeSubsystem(GroundIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    // Run outputs based on current system state
    switch (systemState) {
      case INTAKING:
        io.setIntakeSpeed(1.0);
        io.runIntakePivotToSetpoint(kINTAKING_POSITION_SETPOINT);
        break;
      case REVERSING:
        io.setIntakeSpeed(-1.0);
        break;
      case HOLDING_AT_DEFAULT:
        io.setIntakeSpeed(0.0);
        io.runIntakePivotToSetpoint(kIDLED_POSITION_SETPOINT);
        break;
      case IDLED:
      default:
        io.setIntakeSpeed(0.0);
        io.setPivotSpeed(0.0);
        break;
    }
  }

  private SystemState handleStateTransition() {
    switch (wantedState) {
      case INTAKE:
        return SystemState.INTAKING;
      case REVERSE:
        return SystemState.REVERSING;
      case HOLD_AT_DEFAULT:
        return SystemState.HOLDING_AT_DEFAULT;
      case IDLE:
      default:
        return SystemState.IDLED;
    }
  }

  // Public control methods

  public void stop() {
    setWantedState(WantedState.IDLE);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public Command setWantedStateCommand(WantedState state) {
    return new InstantCommand(() -> setWantedState(state));
  }
}
/* public enum GroundIntakeSubsystem {

}
*/
