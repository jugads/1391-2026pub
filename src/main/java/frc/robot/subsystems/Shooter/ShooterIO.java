package frc.robot.subsystems.Shooter;

public interface ShooterIO {
  default void setShooterSpeed(double speed) {}

  default void setMiddleSpeed(double speed) {}

  default void setFeederSpeed(double speed) {}

  public class shooterIOInputs {

    public double shooterSpeed = 0.0;
    public double middleMotorSpeed = 0.0;
    public double feederSpeed = 0.0;
  }

  default void updateInputs(shooterIOInputs inputs) {}

  default void refreshData() {}
}
