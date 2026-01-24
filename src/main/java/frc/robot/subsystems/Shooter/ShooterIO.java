package frc.robot.subsystems.Shooter;

public interface ShooterIO {
  default void setShooterSpeed(double speed) {}

  default void setWheelSpeed(double speed) {}

  default void setBeltSpeed(double speed) {}

  public class shooterIOInputs {

    public double shooterSpeed = 0.0;
    public double wheelSpeed = 0.0;
    public double beltSpeed = 0.0;
  }

  default void updateInputs(shooterIOInputs inputs) {}

  default void refreshData() {}
}
