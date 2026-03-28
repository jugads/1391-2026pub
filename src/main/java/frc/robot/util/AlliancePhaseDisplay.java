package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AlliancePhaseDisplay {
  private static final double kPhaseLengthSeconds = 25.0;

  private String lastActiveAlliance = "";
  private double phaseStartTime = Timer.getFPGATimestamp();

  public void periodic() {
    String activeAlliance = getActiveAllianceFromGameMessage();
    double now = Timer.getFPGATimestamp();

    // Reset timer whenever the active alliance changes
    if (!activeAlliance.equals(lastActiveAlliance)) {
      lastActiveAlliance = activeAlliance;
      phaseStartTime = now;
    }

    double elapsed = now - phaseStartTime;
    double remaining = Math.max(0.0, kPhaseLengthSeconds - elapsed);

    boolean isMyAllianceActive = isRobotAlliance(activeAlliance);

    SmartDashboard.putString(
      "Alliance State",
      isMyAllianceActive ? "ACTIVE" : "INACTIVE"
    );
    SmartDashboard.putNumber("Alliance Time Remaining", remaining);
  }

  private boolean isRobotAlliance(String activeAlliance) {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    return switch (alliance.get()) {
      case Red -> activeAlliance.equalsIgnoreCase("RED");
      case Blue -> activeAlliance.equalsIgnoreCase("BLUE");
    };
  }

  private String getActiveAllianceFromGameMessage() {
    String msg = DriverStation.getGameSpecificMessage();

    // Replace this with your actual parsing logic
    if (msg.contains("R")) {
      return "RED";
    } else if (msg.contains("B")) {
      return "BLUE";
    }

    return "NONE";
  }
}