package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShoooterConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.shooterIOInputs inputs =
    new ShooterIO.shooterIOInputs();
  public double motorsSetpoint = 0.0;

  public enum WantedState {
    IDLE,
    REV_TO_SPEED,
    SHOOT_AT_HUB,
    REVERSE
  }

  private enum SystemState {
    IDLED,
    REVVING_TO_SPEED,
    SHOOTING_AT_HUB,
    REVERSING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLED;

  private double shooterSpeedSetpoint = 0.0;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    populateInterpolatingDoubleTreeMap();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.refreshData();
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    switch (systemState) {
      case REVVING_TO_SPEED:
        io.setShooterSpeed(motorsSetpoint);
        break;
      case SHOOTING_AT_HUB:
        io.setShooterSpeed(motorsSetpoint);
        io.setFeederSpeed(motorsSetpoint * 0.83 * 0.83);
        break;
      case REVERSING:
        io.setShooterSpeed(kREVERSING_SPEED);
        io.setFeederSpeed(kREVERSING_SPEED);
        break;
      case IDLED:
      default:
        io.setShooterSpeed(0.0);
        io.setFeederSpeed(0.0);
        break;
    }
  }

  private SystemState handleStateTransition() {
    switch (wantedState) {
      case REV_TO_SPEED:
        return SystemState.REVVING_TO_SPEED;
      case SHOOT_AT_HUB:
        return SystemState.SHOOTING_AT_HUB;
      case REVERSE:
        return SystemState.REVERSING;
      case IDLE:
      default:
        return SystemState.IDLED;
    }
  }

  public Command shootCommand(double shooterSpeed) {
    this.motorsSetpoint = shooterSpeed;
    return new InstantCommand(() -> setWantedState(WantedState.REV_TO_SPEED));
  }

  public Command reverse() {
    return new InstantCommand(() -> setWantedState(WantedState.REVERSE));
  }

  public void stop() {
    setWantedState(WantedState.IDLE);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public Command setWantedStateCommand(WantedState state) {
    return new InstantCommand(() -> setWantedState(state));
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public double calculateShooterSpeed(double distance) {
    return kSHOOTER_SPEEDS.get(distance);
  }

  public void populateInterpolatingDoubleTreeMap() {
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_0[0], kSHOOTER_ENTRY_0[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_1[0], kSHOOTER_ENTRY_1[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_2[0], kSHOOTER_ENTRY_2[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_3[0], kSHOOTER_ENTRY_3[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_4[0], kSHOOTER_ENTRY_4[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_5[0], kSHOOTER_ENTRY_5[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_6[0], kSHOOTER_ENTRY_6[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_7[0], kSHOOTER_ENTRY_7[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_8[0], kSHOOTER_ENTRY_8[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_9[0], kSHOOTER_ENTRY_9[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_10[0], kSHOOTER_ENTRY_10[1]);
  }

  public boolean isUpToSpeed() {
    if (Math.abs(shooterSpeedSetpoint - inputs.shooterSpeed) < 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isJammed() {
    // Replace with your actual sensor logic or return false to test
    return false;
  }

  public boolean hasBall() {
    // Replace with sensor or switch input
    return true;
  }

  public Pose3d getShooterPose() {
    return new Pose3d(
      0,0,0, new Rotation3d()
    );
  }
}
