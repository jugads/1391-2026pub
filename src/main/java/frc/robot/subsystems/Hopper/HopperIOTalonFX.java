package frc.robot.subsystems.Hopper;

import static frc.robot.Constants.kCANBUSNAME;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HopperIOTalonFX implements HopperIO {

  private final TalonFX beltMotor;
  
  public HopperIOTalonFX(int beltID) {
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 70;
    currentLimitsConfigs.SupplyCurrentLimit = 40;
    beltMotor = new TalonFX(beltID, kCANBUSNAME);
    beltMotor.getConfigurator().apply(currentLimitsConfigs);
  }

  @Override
  public void setBeltSpeed(double speed) {
    beltMotor.set(speed);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.beltSpeed = beltMotor.get();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    SmartDashboard.putNumber("Hopper/Hopper Speed", beltMotor.get());
  }
}
