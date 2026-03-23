package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {

  private final GroundIntakeIO io;
  private final GroundIntakeIO.GroundIntakeIOInputs inputs =
    new GroundIntakeIO.GroundIntakeIOInputs();
  double setpoint = 0.4;
  double startingTime = 0;
  boolean hasSetStartingWiggleTime = false;

  public enum WantedState {
    IDLE,
    INTAKE,
    HOLD_AT_DEFAULT,
    HOLD_AT_ZERO,
    REVERSE,
    WIGGLE,
  }

  private enum SystemState {
    IDLED,
    INTAKING,
    HOLDING_AT_DEFAULT,
    HOLDING_AT_ZERO,
    REVERSING,
    WIGGLING,
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
    io.refreshData();
    SmartDashboard.putString("Intake/Current State", systemState.toString());
    SmartDashboard.putNumber(
      "time elapsed",
      Timer.getFPGATimestamp() - startingTime
    );
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    // Run outputs based on current system state
    switch (systemState) {
      case INTAKING:
        io.setIntakeSpeed(setpoint);
        io.runIntakePivotToSetpoint(kINTAKING_POSITION_SETPOINT);
        hasSetStartingWiggleTime = false;
        break;
      case REVERSING:
        io.setIntakeSpeed(-1.0);
        hasSetStartingWiggleTime = false;
        break;
      case HOLDING_AT_DEFAULT:
        io.setIntakeSpeed(0.0);
        io.runIntakePivotToSetpoint(kIDLED_POSITION_SETPOINT);
        hasSetStartingWiggleTime = false;
        break;
      case HOLDING_AT_ZERO:
        io.setIntakeSpeed(0.);
        io.runIntakePivotToSetpoint(kZERO_SETPOINT);
        hasSetStartingWiggleTime = false;
        break;
      case IDLED:
        io.setIntakeSpeed(0);
        io.setPivotSpeed(0.);
        hasSetStartingWiggleTime = false;
        break;
      case WIGGLING:
        io.setPivotSpeed(
          (0.35 *
            0.8 *
            Math.signum(
              Math.cos(8 * Math.PI * (Timer.getFPGATimestamp() - startingTime))
            )) +
          0.05
        );
        break;
      default:
        io.setIntakeSpeed(0.0);
        io.runIntakePivotToSetpoint(kZERO_SETPOINT);
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
      case HOLD_AT_ZERO:
        return SystemState.HOLDING_AT_ZERO;
      case WIGGLE:
        return SystemState.WIGGLING;
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

  public Pose3d getIntakePose() {
    return new Pose3d(
      0.0,
      0.0,
      0.0,
      new Rotation3d(0, -inputs.encoderPosition, 0)
    );
  }

  public double getIntakePivotAngle() {
    return inputs.encoderPosition;
  }

  public Command increaseIntakeSpeedSetpoint() {
    return new InstantCommand(() -> setpoint += 0.05);
  }

  public Command resetIntakeSpeedSetpoint() {
    return new InstantCommand(() -> setpoint = 0.45);
  }

  public void wiggle() {
    if (!hasSetStartingWiggleTime) {
      startingTime = Timer.getFPGATimestamp();
      hasSetStartingWiggleTime = true;
    }
    this.wantedState = WantedState.WIGGLE;
  }
}
/* public enum GroundIntakeSubsystem {

}
*/
