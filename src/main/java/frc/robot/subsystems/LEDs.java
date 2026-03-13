package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  PWMSparkMax blinkin;
  WantedState wantedState = WantedState.IDLE;
  CurrentState currentState = CurrentState.IDLING;

  public LEDs(int port) {
    blinkin = new PWMSparkMax(port);
    blinkin.setSafetyEnabled(false);
  }

  public enum WantedState {
    IDLE,
    SHOOT_SPINUP,
    INTAKE,
    SHOOT_READY,
    DISABLED,
    GREEN,
    IDEAL
  }

  public enum CurrentState {
    IDLING,
    SPINNING_UP,
    SHOOTING,
    INTAKING,
    DISABLED,
    PURPLED,
    ABLE_TO_SHOOT
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
      case GREEN:
        currentState = CurrentState.ABLE_TO_SHOOT;
        break;
      case IDEAL:
        currentState = CurrentState.PURPLED;
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
      case ABLE_TO_SHOOT:
        blinkin.set(0.75);
        break;
      case PURPLED:
        blinkin.set(0.91);
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
