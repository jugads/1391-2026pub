package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.ShooterConstants.*;
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private final TalonFX feederMotor;
  private final VelocityVoltage request = new VelocityVoltage(0.);
  public ShooterIOTalonFX(int shooterID,  int feederID) {
    shooterMotor = new TalonFX(shooterID);
    feederMotor = new TalonFX(feederID);
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kG = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    shooterConfig.Slot0 = slot0;

    shooterMotor.getConfigurator().apply(shooterConfig);

  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterMotor.setControl(request.withVelocity(speed));
  }

  @Override
  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Updates the ShooterIOInputs with the current motor speeds.
   * This method is called periodically by the subsystem framework.
   * @param inputs the ShooterIOInputs to update
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterSpeed = shooterMotor.get();
    inputs.feederSpeed = feederMotor.get();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Shooter/Feeder Speed", feederMotor.get());
    SmartDashboard.putNumber("Shooter/Current Draw", shooterMotor.getStatorCurrent().getValueAsDouble());
  }
}
