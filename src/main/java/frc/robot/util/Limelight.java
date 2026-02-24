package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Limelight {

  NetworkTable table;
  String name;

  public Limelight(String limelightName) {
    this.name = limelightName;
    table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    return table.getEntry("ty").getDouble(0.0);
  }

  public boolean isSeeingValidTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public PoseEstimate getLimelightPoseEstimateData() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public void uploadGyro(double angle) {
    LimelightHelpers.SetRobotOrientation(this.name, angle, 0, 0, 0, 0, 0);
  }

  public void setThrottle(double throttle) {
    table.getEntry("throttle_set").setDouble(throttle);
  }

  /**
   * @param mode operating mode selection:
   *             <ul>
   *               <li><b>Mode 1: EXTERNAL_SEED</b> — Uses the robot's external gyro
   *               (via {@code SetRobotOrientation}) and calibrates/seeds the internal
   *               IMU to match it each frame.</li>
   *               <li><b>Mode 2: INTERNAL_ONLY</b> — Relies solely on the Limelight's
   *               internal IMU fused yaw. No external input is required.</li>
   *               <li><b>Mode 3: INTERNAL_MT1_ASSIST</b> — Fuses the internal IMU with
   *               MegaTag1 vision data. Uses the camera pose to correct IMU drift
   *               over time.</li>
   *               <li><b>Mode 4: INTERNAL_EXTERNAL_ASSIST</b> — Recommended and most
   *               stable mode. Fuses the internal 1kHz IMU with the external gyro
   *               (from {@code SetRobotOrientation}) for low-drift, high-rate data.</li>
   *             </ul>
   */

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(this.name, mode);
  }
}
