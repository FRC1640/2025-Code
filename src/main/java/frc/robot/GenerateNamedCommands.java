package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sensors.odometry.RobotOdometry;

public class GenerateNamedCommands {
  public GenerateNamedCommands(RobotOdometry odometry) {
    generateNamedCommands(odometry);
  }

  public void generateNamedCommands(RobotOdometry odometry) {
    NamedCommands.registerCommand(
        "EnableAprilTags", new InstantCommand(() -> odometry.setAutoApriltags(true)));
    NamedCommands.registerCommand(
        "DisableAprilTags", new InstantCommand(() -> odometry.setAutoApriltags(false)));
  }
}
