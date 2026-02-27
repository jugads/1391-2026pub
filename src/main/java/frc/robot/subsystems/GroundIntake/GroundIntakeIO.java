package frc.robot.subsystems.GroundIntake;

public interface GroundIntakeIO {
  public class GroundIntakeIOInputs {

    public double intakeSpeed;
    public double encoderPosition;
    public double pivotSpeed;
  }

  default void updateInputs(GroundIntakeIOInputs inputs) {}

  default void runIntakePivotToSetpoint(double setpoint) {}

  default void refreshData() {}

  default void setIntakeSpeed(double speed) {}

  default void setPivotSpeed(double speed) {}

  default double getIntakePosition() {
    return 0;
  }
}
