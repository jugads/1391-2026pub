package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.kCANBUSNAME;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private final TalonFX feederMotor;
  private final TalonFX shooterMotorFollower;
  private final TalonFX feederMotorFollower;
  private final VelocityVoltage request = new VelocityVoltage(0.);
  private final BangBangController controller = new BangBangController();

  public ShooterIOTalonFX(
    int shooterID,
    int feederID,
    int shooterFollowerID,
    int feederFollowerID
  ) {
    shooterMotor = new TalonFX(shooterID, kCANBUSNAME);
    feederMotor = new TalonFX(feederID, kCANBUSNAME);
    shooterMotorFollower = new TalonFX(shooterFollowerID, kCANBUSNAME);
    feederMotorFollower = new TalonFX(feederFollowerID, kCANBUSNAME);
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 90;
    currentLimitsConfigs.SupplyCurrentLimit = 50;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    shooterConfig.Slot0 = slot0;
    shooterMotor.getConfigurator().apply(shooterConfig);
    feederMotor.getConfigurator().apply(shooterConfig);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterMotor.setControl(request.withVelocity(speed / 60));
    shooterMotorFollower.setControl(
      new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  @Override
  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
    feederMotorFollower.setControl(
      new Follower(feederMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  @Override
  public void stopShooter() {
    shooterMotor.set(0.0);
    shooterMotorFollower.set(0.);
  }

  /**
   * Updates the ShooterIOInputs with the current motor speeds.
   * This method is called periodically by the subsystem framework.
   * @param inputs the ShooterIOInputs to update
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterSpeed = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.feederSpeed = feederMotor.get();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    SmartDashboard.putNumber(
      "Shooter/Shooter Speed",
      shooterMotor.getVelocity().getValueAsDouble() * 60.0
    );
  }
}
