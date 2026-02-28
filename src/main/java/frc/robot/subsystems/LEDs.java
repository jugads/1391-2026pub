package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  Spark blinkin;
  WantedState wantedState = WantedState.IDLE;
  CurrentState currentState = CurrentState.IDLING;

  public LEDs(int port) {
    blinkin = new Spark(port);
  }

  public enum WantedState {
    IDLE,
    SHOOT_SPINUP,
    INTAKE,
    SHOOT_READY,
    DISABLED
  }

  public enum CurrentState {
    IDLING,
    SPINNING_UP,
    SHOOTING,
    INTAKING,
    DISABLED
  }

  @Override
  public void periodic() {
    switch (wantedState) {
      case IDLE:
        currentState = CurrentState.IDLING;
        break;
      case SHOOT_SPINUP:
        currentState = CurrentState.SHOOTING;
        break;
      case INTAKE:
        currentState = CurrentState.INTAKING;
        break;
      case SHOOT_READY:
        currentState = CurrentState.SHOOTING;
        break;
      case DISABLED:
        currentState = CurrentState.DISABLED;
        break;
    }

    switch (currentState) {
      case IDLING:
        blinkin.set(0.99);
        break;
      case SHOOTING:
        blinkin.set(-0.07);
        break;
      case SPINNING_UP:
        blinkin.set(0.67);
        break;
      case INTAKING:
        blinkin.set(-0.23);
        break;
      case DISABLED:
        blinkin.set(0.61);
        break;
    }
  }

  public Command setWantedStateCommand(WantedState state) {
    return new InstantCommand(() -> wantedState = state);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void stop() {
    setWantedState(WantedState.IDLE);
  }
}
