package frc.robot.util;

import static frc.robot.Constants.AutonomousConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    initiateNamedCommands();
  }

  public void initiateNamedCommands() {
    WantedSuperState[] states = WantedSuperState.values();
    for (int i = 0; i < states.length; i++) {
      NamedCommands.registerCommand(
        "Set Robot State To " + states[i].toString(),
        m_robot.setWantedSuperStateCommand(states[i])
      );
      System.out.println(
        "ROB-state_" + states[i].toString()
      );
    }
    NamedCommands.registerCommand(
      "INT-wait_trench",
      new WaitUntilCommand(() -> m_robot.canDriveUnderTrenchSafely())
    );

  }

  public SendableChooser<PathPlannerAuto> register(SendableChooser<PathPlannerAuto> chooser) {
    chooser.addOption("RS-Main Cycle + Outpost", new PathPlannerAuto("RS-main"));
    return chooser;
  }
}
