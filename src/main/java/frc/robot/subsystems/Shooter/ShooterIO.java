package frc.robot.subsystems.Shooter;

public interface ShooterIO {
  default void setShooterSpeed(double speed) {}


  default void setFeederSpeed(double speed) {}

  public class ShooterIOInputs {

    public double shooterSpeed = 0.0;
    public double feederSpeed = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void refreshData() {}
}
