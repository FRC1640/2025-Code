package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.weights.PathplannerWeight;

public class GenerateNamedCommands {
  public GenerateNamedCommands(RobotOdometry odometry) {
    generateNamedCommands(odometry);
  }

  public void generateNamedCommands(RobotOdometry odometry) {
    NamedCommands.registerCommand(
        "EnableAprilTags", new InstantCommand(() -> odometry.setAutoApriltags(true)));
    NamedCommands.registerCommand(
        "DisableAprilTags", new InstantCommand(() -> odometry.setAutoApriltags(false)));
    NamedCommands.registerCommand(
        "DisableRotation", new InstantCommand(() -> PathplannerWeight.overrideRotation(() -> 0)));
    NamedCommands.registerCommand(
        "NormalRotation", new InstantCommand(() -> PathplannerWeight.clearRotationOverride()));
  }
}
