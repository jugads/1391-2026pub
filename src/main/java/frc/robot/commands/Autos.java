package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Constants.AutonomousConstants.*;
public class Autos {
    CommandSwerveDrivetrain drivetrain;
    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    public Command rightSideAuto() {
        PathPlannerPath getFuelPath;
        try {
            getFuelPath = PathPlannerPath.fromPathFile("RS-getFuel");
        } catch (Exception e) {
            e.printStackTrace();
            getFuelPath = null;
        }
        return Commands.sequence(
            new InstantCommand(() -> drivetrain.resetGlobalPose(kSTARTING_POSE_RIGHT_SIDE)),
            AutoBuilder.followPath(getFuelPath)

        );
    }
}
