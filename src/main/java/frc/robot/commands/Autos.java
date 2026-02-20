package frc.robot.commands;

import static frc.robot.Constants.AutonomousConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotCore;
import frc.robot.RobotCore.WantedSuperState;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autos {

  CommandSwerveDrivetrain drivetrain;
  PathPlannerPath getFuelPath;
  RobotCore m_robot;
  public Autos(RobotCore robotCore) {
    this.drivetrain = robotCore.fetchDrivetrain();
    drivetrain.configureAutoBuilder();
    m_robot = robotCore;
    try {
      getFuelPath = PathPlannerPath.fromPathFile("RS-getFuel1");
    } catch (Exception e) {
      e.printStackTrace();
      getFuelPath = null;
      System.out.println("Path not found");
    }
  }

  public Command rightSideAuto() {
    return Commands.sequence(
      new InstantCommand(() ->
        drivetrain.resetGlobalPose(kSTARTING_POSE_RIGHT_SIDE)
      ),
      m_robot.setWantedSuperStateCommand(WantedSuperState.INTAKE),
      new WaitUntilCommand(() -> m_robot.canDriveUnderTrenchSafely()),
      AutoBuilder.followPath(getFuelPath)
    );
  }
}
