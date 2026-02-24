package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private final TalonFX middleMotor;
  private final TalonFX feederMotor;

  public ShooterIOTalonFX(int shooterID, int middleMotorID,  int feederID) {
    shooterMotor = new TalonFX(shooterID);
    middleMotor = new TalonFX(middleMotorID);
    feederMotor = new TalonFX(feederID);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setFeederSpeed(double speed) {
    shooterMotor.set(speed);
  }

  /**
   * Updates the ShooterIOInputs with the current motor speeds.
   * This method is called periodically by the subsystem framework.
   * @param inputs the ShooterIOInputs to update
   */
  @Override
  public void updateInputs(shooterIOInputs inputs) {
    inputs.shooterSpeed = shooterMotor.get();
    inputs.feederSpeed = feederMotor.get();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Shooter/Wheel Speed", middleMotor.get());
    SmartDashboard.putNumber("Shooter/Belt Speed", feederMotor.get());
    SmartDashboard.putNumber("Shooter/Current Draw", shooterMotor.getStatorCurrent().getValueAsDouble());
  }
}
