package frc.robot.subsystems.Shooter;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shooter IO simulation.
 *
 * Important: This matches your current real implementation behavior:
 * - setShooterSpeed/setWheelSpeed/setBeltSpeed treat "speed" as percent output [-1..1]
 * - updateInputs returns those percent outputs (like TalonFX.get() does)
 *
 * We also run FlywheelSim internally so you can view simulated RPM in SmartDashboard.
 */
public class ShooterIOSim implements ShooterIO {

  // Percent outputs (what your subsystem is effectively commanding today)
  private double shooterPercent = 0.0;
  private double wheelPercent = 0.0;
  private double beltPercent = 0.0;
  private Pose3d shooterPose = new Pose3d(
    0.0, 0.0, 0.0,
    new Rotation3d()
  );
  // Simple sims (tweak MOI + gearing to better match your mechanism)
  private final FlywheelSim shooterSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.020, 0.0010),
    DCMotor.getFalcon500(1),
    1.0
  );

  private final FlywheelSim wheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.018, 0.0010),
    DCMotor.getFalcon500(1),
    1.0
  );

  private final FlywheelSim beltSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.030, 0.0015),
    DCMotor.getFalcon500(1),
    1.0
  );
  private double lastTimestamp = Timer.getFPGATimestamp();

  @Override
  public void setShooterSpeed(double speed) {
    shooterPercent = clamp(speed, -1.0, 1.0);
  }

  @Override
  public void setMiddleSpeed(double speed) {
    wheelPercent = clamp(speed, -1.0, 1.0);
  }

  @Override
  public void setFeederSpeed(double speed) {
    beltPercent = clamp(speed, -1.0, 1.0);
  }

  @Override
  public void updateInputs(shooterIOInputs inputs) {
    // Advance sims
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    // Protect against weird dt on first cycle
    if (dt <= 0 || dt > 0.1) {
      dt = 0.02;
    }

    double batt = RobotController.getBatteryVoltage();

    shooterSim.setInputVoltage(shooterPercent * batt);
    wheelSim.setInputVoltage(wheelPercent * batt);
    beltSim.setInputVoltage(beltPercent * batt);

    shooterSim.update(dt);
    wheelSim.update(dt);
    beltSim.update(dt);

    // Match your current TalonFX IO behavior: return commanded percent outputs
    inputs.shooterSpeed = shooterPercent;
    inputs.middleMotorSpeed = wheelPercent;
    inputs.feederSpeed = beltPercent;
  }

  @Override
  public void refreshData() {
    // Percent outputs (matches inputs)
    SmartDashboard.putNumber("ShooterSim/Shooter Percent", shooterPercent);
    SmartDashboard.putNumber("ShooterSim/Wheel Percent", wheelPercent);
    SmartDashboard.putNumber("ShooterSim/Belt Percent", beltPercent);

    // Extra sim telemetry (not part of ShooterIOInputs, but helpful)
    SmartDashboard.putNumber(
      "ShooterSim/Shooter RPM",
      radPerSecToRpm(shooterSim.getAngularVelocityRadPerSec())
    );
    SmartDashboard.putNumber(
      "ShooterSim/Wheel RPM",
      radPerSecToRpm(wheelSim.getAngularVelocityRadPerSec())
    );
    SmartDashboard.putNumber(
      "ShooterSim/Belt RPM",
      radPerSecToRpm(beltSim.getAngularVelocityRadPerSec())
    );

    SmartDashboard.putNumber(
      "ShooterSim/Shooter Current (A)",
      shooterSim.getCurrentDrawAmps()
    );
    SmartDashboard.putNumber(
      "ShooterSim/Wheel Current (A)",
      wheelSim.getCurrentDrawAmps()
    );
    SmartDashboard.putNumber(
      "ShooterSim/Belt Current (A)",
      beltSim.getCurrentDrawAmps()
    );
}

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private static double radPerSecToRpm(double radPerSec) {
    return (radPerSec * 60.0) / (2.0 * Math.PI);
  }
}
