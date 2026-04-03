// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.TidalLock;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TidalLockCommand extends Command {

  /** Creates a new TidalLockCommand. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.FieldCentric request;
  TidalLock tidalLock = new TidalLock();
  DoubleSupplier xControl;
  DoubleSupplier yControl;
  DoubleSupplier dTheta;

  public TidalLockCommand(
    CommandSwerveDrivetrain drivetrain,
    DoubleSupplier xControl,
    DoubleSupplier yControl,
    SwerveRequest.FieldCentric request,
    DoubleSupplier dTheta
  ) {
    this.drivetrain = drivetrain;
    this.xControl = xControl;
    this.yControl = yControl;
    this.request = request;
    this.dTheta = dTheta;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Dtheta", dTheta.getAsDouble());
    drivetrain.setControl(
      request
        .withRotationalRate(
          tidalLock.getOutput(
            drivetrain.getPigeon2().getRotation2d().getDegrees(),
            drivetrain.getAngleToHub().getDegrees(),
            0 // drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond
          ) +
          dTheta.getAsDouble() *
          0.1 *
          RotationsPerSecond.of(0.75).in(RadiansPerSecond)
        )
        .withVelocityX(-xControl.getAsDouble() * 0.7)
        .withVelocityY(-yControl.getAsDouble() * 0.7)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
