package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs =
    new ShooterIO.ShooterIOInputs();
  public int cyclesOfShooterUpToSpeed;
  public double motorsSetpoint = 0.0;


  public enum WantedState {
    IDLE,
    REV_TO_SPEED,
    SHOOT_AT_HUB,
    REVERSE,
    FEED_AND_SHOOT,
    WARM_UP
  }

  private enum SystemState {
    IDLED,
    REVVING_TO_SPEED,
    SHOOTING_AT_HUB,
    REVERSING,
    FEEDING_AND_SHOOTING,
    WARMING_UP
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
    if (
      Math.abs(inputs.shooterSpeed - motorsSetpoint) <
        kSHOOTER_SPEED_TOLERANCE &&
      motorsSetpoint != 0
    ) {
      cyclesOfShooterUpToSpeed++;
    } else {
      cyclesOfShooterUpToSpeed = 0;
    }
    SmartDashboard.putString("Shooter/WantedState", wantedState.toString());
    SmartDashboard.putNumber("Shooter/MotorsSetpoint", motorsSetpoint);
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    switch (systemState) {
      case REVVING_TO_SPEED:
        io.setShooterSpeed(motorsSetpoint);
        break;
      case WARMING_UP:
        io.setShooterSpeed(4000);
        io.setFeederSpeed(0.0);
        break;
      case SHOOTING_AT_HUB:
        io.setShooterSpeed(motorsSetpoint);
        io.setFeederSpeed(1.);
        break;
      case REVERSING:
        io.setShooterSpeed(kREVERSING_SPEED);
        io.setFeederSpeed(-0.7);
        break;
      case FEEDING_AND_SHOOTING:
        io.setShooterSpeed(motorsSetpoint);
        io.setFeederSpeed(1);
        break;
      case IDLED:
      default:
        io.stopShooter();
        io.setFeederSpeed(0.0);
        break;
    }
  }

  private SystemState handleStateTransition() {
    switch (wantedState) {
      case REV_TO_SPEED:
        return SystemState.REVVING_TO_SPEED;
      case WARM_UP:
        return SystemState.WARMING_UP;
      case SHOOT_AT_HUB:
        return SystemState.SHOOTING_AT_HUB;
      case REVERSE:
        return SystemState.REVERSING;
      case FEED_AND_SHOOT:
        return SystemState.FEEDING_AND_SHOOTING;
      case IDLE:
      default:
        return SystemState.IDLED;
    }
  }

  public Command shootCommand(double shooterSpeed) {
    this.motorsSetpoint = shooterSpeed;
    return new InstantCommand(() -> setWantedState(WantedState.REV_TO_SPEED));
  }

  public void shoot(double shooterSpeed) {
    this.motorsSetpoint = shooterSpeed;
    setWantedState(WantedState.REV_TO_SPEED);
  }

  public void shootAtHub() {
    this.motorsSetpoint = SmartDashboard.getNumber("shooter speed", 5000) + 150;
    setWantedState(WantedState.SHOOT_AT_HUB);
  }
  public void shootAtHubRegular() {
    this.motorsSetpoint = kSHOOTER_SPEED_AT_HUB;
    setWantedState(WantedState.SHOOT_AT_HUB);
  }
  public void feedAndShoot(double setpoint) {
    this.motorsSetpoint = setpoint;
    setWantedState(WantedState.FEED_AND_SHOOT);
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
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_000[0], kSHOOTER_ENTRY_000[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_001[0], kSHOOTER_ENTRY_001[1]);
    kSHOOTER_SPEEDS.put(kSHOOTER_ENTRY_0002[0], kSHOOTER_ENTRY_0002[1]);
  }

  public boolean isUpToSpeed() {
    return cyclesOfShooterUpToSpeed > 10;
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
    return new Pose3d(0, 0, 0, new Rotation3d());
  }
}
