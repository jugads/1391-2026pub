package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;

import com.ctre.phoenix6.hardware.*;

public interface GroundIntakeIO {
  public class GroundIntakeIOInputs {

    public double intakeSpeed;
    public double encoderPosition;
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
