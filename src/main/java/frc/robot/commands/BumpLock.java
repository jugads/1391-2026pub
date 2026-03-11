package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class BumpLock extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier xControl;
  private final DoubleSupplier yControl;

  private final PIDController thetaController = new PIDController(
    0.08,
    0.0,
    0.0
  );
  private final SwerveRequest.FieldCentric request;

  private double lockedBumpAngleDeg;

  public BumpLock(
    CommandSwerveDrivetrain drivetrain,
    DoubleSupplier xControl,
    DoubleSupplier yControl,
    SwerveRequest.FieldCentric request
  ) {
    this.drivetrain = drivetrain;
    this.xControl = xControl;
    this.yControl = yControl;
    this.request = request;

    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(8);
    addRequirements(drivetrain);
  }

  public enum TravelDirection {
    GOING_THERE,
    COMING_BACK,
  }

  public enum FieldSide {
    LEFT,
    RIGHT,
  }

  public double getBumpAngleDeg() {
    FieldSide side;
    Alliance alliance = (this.drivetrain.getGlobalPose().getX() < 8.3)
      ? Alliance.Blue
      : Alliance.Red;
    TravelDirection direction =
      (this.drivetrain.isInAllianceZone(alliance)
          ? TravelDirection.GOING_THERE
          : TravelDirection.COMING_BACK);
    if (alliance == Alliance.Blue) {
      side = this.drivetrain.getGlobalPose().getY() > 4.0
        ? FieldSide.LEFT
        : FieldSide.RIGHT;
    } else {
      side = this.drivetrain.getGlobalPose().getY() > 4.0
        ? FieldSide.RIGHT
        : FieldSide.LEFT;
    }
    if (alliance == Alliance.Blue) {
      if (side == FieldSide.LEFT) {
        return direction == TravelDirection.GOING_THERE ? -135 : -45;
      } else {
        return direction == TravelDirection.GOING_THERE ? 135 : 45;
      }
    } else {
      if (side == FieldSide.LEFT) {
        return direction == TravelDirection.GOING_THERE ? -45 : 135;
      } else {
        return direction == TravelDirection.GOING_THERE ? 45 : -135.0;
      }
    }
  }

  public double snap(double input) {
    if (Math.abs(input) < 0.1) {
      return 0.;
    }
    return Math.signum(input);
  }

  @Override
  public void initialize() {
    lockedBumpAngleDeg = getBumpAngleDeg();
    thetaController.reset();
  }

  @Override
  public void execute() {
    drivetrain.setControl(
      request
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withVelocityX(snap(yControl.getAsDouble()) * 2.0)
        .withVelocityY(xControl.getAsDouble() * 5.12 * 0.25)
        .withRotationalRate(
          thetaController.calculate(
            drivetrain.getGlobalPose().getRotation().getDegrees(),
            lockedBumpAngleDeg
          )
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
      request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
