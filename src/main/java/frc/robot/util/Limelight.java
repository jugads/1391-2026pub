package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.LimelightHelpers.PoseEstimate;

import java.util.Optional;

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
}