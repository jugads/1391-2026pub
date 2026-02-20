package frc.robot.subsystems.Hopper;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Lightweight sim implementation for HopperIO.
 * - Treats setBeltSpeed(speed) as percent output in [-1, 1].
 * - Reports a ramped/filtered "beltSpeed" to mimic motor/belt inertia.
 */
public class HopperIOSim implements HopperIO {
  // Commanded percent output
  private double commandedSpeed = 0.0;

  // Simulated/filtered percent output (what we report back)
  private double simulatedSpeed = 0.0;

  // How fast the belt can change (units: percent-output per second)
  private final SlewRateLimiter accelLimiter;

  public HopperIOSim() {
    this(6.0); // default ramp rate
  }

  public HopperIOSim(double rampRatePerSec) {
    accelLimiter = new SlewRateLimiter(rampRatePerSec);
  }

  @Override
  public void setBeltSpeed(double speed) {
    // Clamp to typical percent output range
    commandedSpeed = clamp(speed, -1.0, 1.0);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    simulatedSpeed = accelLimiter.calculate(commandedSpeed);
    inputs.beltSpeed = simulatedSpeed;
  }

  @Override
  public void refreshData() {
    SmartDashboard.putNumber("Hopper/CommandedSpeed", commandedSpeed);
    SmartDashboard.putNumber("Hopper/SimBeltSpeed", simulatedSpeed);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}