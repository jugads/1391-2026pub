package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Physics-based sim for the Ground Intake pivot + intake roller.
 *
 * Notes:
 * - Your real IO uses a DutyCycleEncoder and reports:
 *     encoderPosition = encoder.get() - kENCODER_OFFSET
 *   We mimic that by mapping the arm angle to a "raw encoder" in [0..1),
 *   then subtracting kENCODER_OFFSET when reporting.
 *
 * - A 4-bar can be visualized accurately in AdvantageScope ONLY if you compute
 *   and publish Pose3d(s) for each link. This sim focuses on the pivot joint
 *   (equivalent angle). You can use that angle to drive your 3D poses.
 */
public class GroundIntakeIOSim implements GroundIntakeIO {

  // ----- Tune these to match your mechanism -----
  // Pivot geometry approximation (treat 4-bar as a single equivalent pivot).
  // These limits define how encoder units map to physical angle.
  private static final double kMinAngleRad = Math.toRadians(0.0);  // intake stowed-ish
  private static final double kMaxAngleRad = Math.toRadians(120.0);  // intake deployed-ish

  // Map angle <-> encoderPosition (after offset). If your setpoints are not 0..1,
  // adjust these.
  private static final double kMinEncoderPos = 0.0;   // (encoder.get() - offset) at min angle
  private static final double kMaxEncoderPos = 1.0;   // (encoder.get() - offset) at max angle

  // Pivot sim physical params (rough defaults; tune later)
  private static final double kPivotGearRatio = 60.0;     // motor rotations per arm rotation
  private static final double kArmLengthMeters = 0.35;    // effective COM radius
  private static final double kArmMoi = 0.02;             // kg*m^2 effective inertia

  // Intake roller sim params (optional but nice)
  private static final double kRollerGearRatio = 1.0;

  // ----- Commanded outputs -----
  private double intakePercent = 0.0;
  private double pivotPercent = 0.0;

  // ----- Control -----
  private final PIDController positionPid = new PIDController(kP, kI, kD);
  private boolean closedLoop = false;
  private double positionSetpoint = 0.0;

  // ----- Sims -----
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          kPivotGearRatio,
          kArmMoi,
          kArmLengthMeters,
          kMinAngleRad,
          kMaxAngleRad,
          true,               // simulate gravity
          angleFromEncoderPos(kIDLED_POSITION_SETPOINT) // initial angle from your constants
      );

  private final FlywheelSim rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.02, 0.001),
          DCMotor.getKrakenX60(1),
          kRollerGearRatio
      );

  private double lastTs = Timer.getFPGATimestamp();

  @Override
  public void setIntakeSpeed(double speed) {
    intakePercent = MathUtil.clamp(speed, -1.0, 1.0);
  }

  @Override
  public void setPivotSpeed(double speed) {
    // open-loop mode
    closedLoop = false;
    pivotPercent = MathUtil.clamp(speed, -1.0, 1.0);
  }

  @Override
  public void runIntakePivotToSetpoint(double setpoint) {
    // closed-loop mode (mimics your real IO behavior) :contentReference[oaicite:2]{index=2}
    closedLoop = true;
    positionSetpoint = setpoint;
    positionPid.setSetpoint(setpoint);

    // We'll compute pivotPercent inside updateInputs using current sim position.
  }

  @Override
  public double getIntakePosition() {
    // This should behave like: encoder.get() - kENCODER_OFFSET :contentReference[oaicite:3]{index=3}
    return encoderPosFromAngle(pivotSim.getAngleRads());
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTs;
    lastTs = now;
    if (dt <= 0 || dt > 0.1) dt = 0.02;

    // Closed-loop pivot control: PID on encoderPosition units (same as your real code)
    if (closedLoop) {
      double currentPos = getIntakePosition();
      double out = positionPid.calculate(currentPos);
      pivotPercent = MathUtil.clamp(out, -1.0, 1.0);
    }

    // Apply voltages
    double batt = RobotController.getBatteryVoltage();

    pivotSim.setInputVoltage(pivotPercent * batt);
    pivotSim.update(dt);

    rollerSim.setInputVoltage(intakePercent * batt);
    rollerSim.update(dt);

    // Publish inputs expected by subsystem :contentReference[oaicite:4]{index=4}
    inputs.intakeSpeed = intakePercent;          // matches TalonFX.get() style :contentReference[oaicite:5]{index=5}
    inputs.encoderPosition = getIntakePosition();    // matches encoder.get()-offset :contentReference[oaicite:6]{index=6}
  }

  @Override
  public void refreshData() {
    SmartDashboard.putNumber("GroundIntakeSim/IntakePercent", intakePercent);
    SmartDashboard.putNumber("GroundIntakeSim/PivotPercent", pivotPercent);
    SmartDashboard.putNumber("GroundIntakeSim/EncoderPosition", getIntakePosition());
    SmartDashboard.putNumber("GroundIntakeSim/PivotAngleDeg", Math.toDegrees(pivotSim.getAngleRads()));
    SmartDashboard.putNumber("GroundIntakeSim/PivotCurrentA", pivotSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("GroundIntakeSim/RollerRPM", rollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI));
    SmartDashboard.putNumber("GroundIntakeSim/RollerCurrentA", rollerSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("GroundIntakeSim/Setpoint", positionSetpoint);
  }

  // ----- Mapping helpers -----

  /** Convert encoderPosition (encoder.get()-offset) to an angle in radians. */
  private static double angleFromEncoderPos(double encoderPos) {
    double t = (encoderPos - kMinEncoderPos) / (kMaxEncoderPos - kMinEncoderPos);
    t = MathUtil.clamp(t, 0.0, 1.0);
    return kMinAngleRad + t * (kMaxAngleRad - kMinAngleRad);
  }

  /** Convert angle radians to encoderPosition (encoder.get()-offset). */
  private static double encoderPosFromAngle(double angleRad) {
    double t = (angleRad - kMinAngleRad) / (kMaxAngleRad - kMinAngleRad);
    t = MathUtil.clamp(t, 0.0, 1.0);
    return kMinEncoderPos + t * (kMaxEncoderPos - kMinEncoderPos);
  }
}