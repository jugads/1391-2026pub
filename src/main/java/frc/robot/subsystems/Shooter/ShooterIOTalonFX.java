package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private final TalonFX wheelMotor;
  private final TalonFX beltMotor;

  public ShooterIOTalonFX(int motorID, int wheelID, int beltID) {
    shooterMotor = new TalonFX(motorID);
    wheelMotor = new TalonFX(wheelID);
    beltMotor = new TalonFX(beltID);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setWheelSpeed(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setBeltSpeed(double speed) {
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
    inputs.wheelSpeed = wheelMotor.get();
    inputs.beltSpeed = beltMotor.get();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Shooter/Wheel Speed", wheelMotor.get());
    SmartDashboard.putNumber("Shooter/Belt Speed", beltMotor.get());
  }
}
