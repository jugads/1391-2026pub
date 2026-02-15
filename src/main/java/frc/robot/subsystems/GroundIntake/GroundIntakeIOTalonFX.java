package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeIOTalonFX implements GroundIntakeIO {

  private final TalonFX pivotMotor;
  private final TalonFX intakeMotor;
  private final TalonFX intakeMotorFollower;
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  public GroundIntakeIOTalonFX(int pivotID, int intakeID, int followerID) {
    this.pivotMotor = new TalonFX(pivotID);
    this.intakeMotor = new TalonFX(intakeID);
    this.intakeMotorFollower = new TalonFX(followerID);
    pivotMotor.setPosition(0);
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    pivotMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    this.intakeMotor.set(speed);
    this.intakeMotorFollower.set(speed);
  }

  @Override
  public void setPivotSpeed(double speed) {
    this.pivotMotor.set(speed);
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    //SPARK MAX VERSION: inputs.intakeSpeed = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeSpeed = intakeMotor.get();
    //SPARK MAX VERSION: inputs.encoderPosition = pivotMotor.getOutputCurrent();
    inputs.encoderPosition = pivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/WheelSpeed", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/EncoderPosition", pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("GroundIntake/WheelSpeed", intakeMotor.get());
    SmartDashboard.putNumber(
      "GroundIntake/EncoderPosition",
      getIntakePosition()
    );
  }

  /* @Override
    public double getEncoderVal() {
      return this.pivotMotor.getEncoder().getPosition();
    } */

  @Override
  public double getIntakePosition() {
    // getPosition() returns a StatusSignal; getValueAsDouble() converts it to a numeric value.
    return pivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void runIntakePivotToSetpoint(double setpoint) {
    pivotMotor.setControl(motionMagicVoltage.withPosition(setpoint));
  }
}
