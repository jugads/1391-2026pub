// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SubsurfaceDash extends Command {

  /** Creates a new SubsurfaceDash. */
  CommandSwerveDrivetrain drivetrain;
  PIDController xController = new PIDController(0, 0, 0);
  PIDController yController = new PIDController(0., 0, 0);
  PIDController thetaController = new PIDController(0., 0, 0);
  SwerveRequest.RobotCentric request = new RobotCentric();
  Limelight camera;

  public SubsurfaceDash(
    CommandSwerveDrivetrain drivetrain,
    SwerveRequest.RobotCentric request,
    Limelight camera
  ) {
    this.drivetrain = drivetrain;
    this.request = request;
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance alliance = DriverStation.getAlliance().get();
    if (drivetrain.isInAllianceZone(alliance)) {
      xController.setSetpoint(kTRENCH_GOING_OUT_SETPOINTS[0]);
      yController.setSetpoint(kTRENCH_GOING_OUT_SETPOINTS[1]);
      thetaController.setSetpoint(
        kTRENCH_GOING_OUT_SETPOINTS[2] + (alliance == Alliance.Blue ? 0 : 180)
      );
    } else {
      xController.setSetpoint(kTRENCH_GOING_IN_SETPOINTS[0]);
      yController.setSetpoint(kTRENCH_GOING_IN_SETPOINTS[1]);
      thetaController.setSetpoint(
        kTRENCH_GOING_IN_SETPOINTS[2] + (alliance == Alliance.Blue ? 0 : 180)
      );
    }
    xController.setTolerance(2.);
    yController.setTolerance(2.);
    thetaController.setTolerance(2.);
    SmartDashboard.putData(xController);
    SmartDashboard.putData(yController);
    SmartDashboard.putData(thetaController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!xController.atSetpoint() || !yController.atSetpoint()) {
      drivetrain.setControl(
        request
          .withVelocityX(xController.calculate(camera.getTY()))
          .withVelocityY(yController.calculate(camera.getTX()))
          .withRotationalRate(
            thetaController.calculate(
              drivetrain.getGlobalPose().getRotation().getDegrees()
            )
          )
      );
    }
    else {
      drivetrain.setControl(request.withVelocityX(2.).withVelocityY(0).withRotationalRate(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
