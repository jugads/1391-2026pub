// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotCore.WantedSuperState;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOTalonFX;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.Hopper.HopperIOSim;
import frc.robot.subsystems.Hopper.HopperIOTalonFX;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.Limelight;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.MotorIDConstants.*;
public class RobotContainer {

  private double MaxSpeed =
    1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(
    RadiansPerSecond
  ); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
    new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric driveRR =
    new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain =
    TunerConstants.createDrivetrain();

  private final ShooterSubsystem shooter = new ShooterSubsystem(
    new ShooterIOTalonFX(kSHOOTER_ID, kLOADER_ID)
  );

  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem(
    new GroundIntakeIOTalonFX(kPIVOT_ID, kINTAKE_ID1, kINTAKE_ID2)
  );
  private final HopperSubsystem hopper = new HopperSubsystem(new HopperIOTalonFX(kHOP_ID));

  private final LEDs leds = new LEDs(kLED_PORT);

  private final RobotCore robotSuper = new RobotCore(
    shooter,
    groundIntake,
    hopper,
    drivetrain,
    leds
  );

  private final Limelight tagLimelight = new Limelight("limelight-intake");
  private final Autos autos = new Autos(robotSuper);

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Global Pose", Pose2d.struct)
    .publish();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(
        () ->
          drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    joystick
      .leftTrigger()
      .whileTrue(robotSuper.setWantedSuperStateCommand(WantedSuperState.INTAKE))
      .whileFalse(robotSuper.setWantedSuperStateCommand(WantedSuperState.HOME));
    joystick
      .rightTrigger()
      .whileTrue(robotSuper.setWantedSuperStateCommand(WantedSuperState.SHOOT))
      .whileFalse(robotSuper.setWantedSuperStateCommand(WantedSuperState.HOME));
    joystick
      .rightBumper()
      .whileTrue(robotSuper.setIntakeOverrideCommand(true))
      .whileFalse(robotSuper.setIntakeOverrideCommand(false));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    return autos.rightSideAuto();
  }

  public void dashboardUpdates() {
    // tagLimelight.uploadGyro(
    //   drivetrain.getPigeon2().getRotation2d().getDegrees()
    // );

    // publisher.set(drivetrain.getGlobalPose());

    // if (
    //   tagLimelight.getLimelightPoseEstimateData() != null &&
    //   tagLimelight.getLimelightPoseEstimateData().tagCount > 0
    // ) {
    //   drivetrain.updateGlobalPoseWithVisionMeasurements(
    //     tagLimelight.getLimelightPoseEstimateData().pose,
    //     tagLimelight.getLimelightPoseEstimateData().timestampSeconds
    //   );
    // }
    // if (!DriverStation.getAlliance().isEmpty()) {
    //   if (drivetrain.isInAllianceZone(DriverStation.getAlliance().get())) {
    //     leds.setWantedState(LEDs.WantedState.ZONE_ASSIST);
    //   }
    // }
    // SmartDashboard.putNumber(
    //   "Angle to Hub",
    //   drivetrain.getAngleToHub().getDegrees()
    // );

    // Logger.recordOutput(
    //   "Robot/ComponentPoses",
    //   new Pose3d[] { shooter.getShooterPose(), groundIntake.getIntakePose() }
    // );
  }
}
