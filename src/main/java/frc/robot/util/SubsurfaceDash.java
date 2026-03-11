package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SubsurfaceDash {

  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController thetaController = new PIDController(
    0.025,
    0.0,
    0.0
  );
  private double lockedHeadingDeg;

  public SubsurfaceDash(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    thetaController.enableContinuousInput(-180.0, 180.0);
  }

  public void initialize() {
    lockedHeadingDeg = 0;
    thetaController.reset();
  }

  public double calculateRotationalVelocity() {
    double speed = thetaController.calculate(
      drivetrain.getGlobalPose().getRotation().getDegrees(),
      lockedHeadingDeg
    );
    SmartDashboard.putNumber("Theta calculation", speed);

    return speed;
  }

  public double getBumpAngleDeg() {
    return 0;
  }
}
