package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  Spark blinkin;
  WantedState wantedState = WantedState.IDLE;
  CurrentState currentState = CurrentState.IDLING;
  ZoneAssist zoneAssist = ZoneAssist.FAR;

  public LEDs(int port) {
    blinkin = new Spark(port);
  }

  public enum WantedState {
    IDLE,
    SHOOT,
    INTAKE,
    ZONE_ASSIST,
  }

  public enum CurrentState {
    IDLING,
    SHOOTING,
    INTAKING,
    ZONE_ASSISTING,
  }

  public enum ZoneAssist {
    FAR,
    MEDIUM,
    NEAR,
  }

  @Override
  public void periodic() {
    switch (wantedState) {
      case IDLE:
        currentState = CurrentState.IDLING;
        break;
      case SHOOT:
        currentState = CurrentState.SHOOTING;
        break;
      case INTAKE:
        currentState = CurrentState.INTAKING;
        break;
      case ZONE_ASSIST:
        currentState = CurrentState.ZONE_ASSISTING;
        break;
    }

    switch (currentState) {
      case IDLING:
        blinkin.set(0.99);
        break;
      case SHOOTING:
        blinkin.set(-0.07);
        break;
      case INTAKING:
        blinkin.set(-0.23);
        break;
      case ZONE_ASSISTING:
        switch (zoneAssist) {
          case FAR:
            blinkin.set(0.61);
            break;
          case MEDIUM:
            blinkin.set(0.93);
            break;
          case NEAR:
            blinkin.set(0.75);
            break;
        }
        break;
    }
  }

  public Command setWantedStateCommand(WantedState state) {
    return new InstantCommand(() -> wantedState = state);
  }

  public void setZoneAssist(ZoneAssist zoneAssist) {
    this.zoneAssist = zoneAssist;
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void stop() {
    setWantedState(WantedState.IDLE);
  }
}
