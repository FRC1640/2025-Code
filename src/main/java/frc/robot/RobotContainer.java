// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIOPhotonvision;
import frc.robot.sensors.apriltag.AprilTagVisionIOSim;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.gyro.GyroIONavX;
import frc.robot.sensors.gyro.GyroIOSim;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import java.util.ArrayList;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;
  private final RobotOdometry robotOdometry;
  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  public RobotContainer() {
    switch (Robot.getMode()) {
      case REAL:
        gyro = new Gyro(new GyroIONavX());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision("Front", CameraConstants.frontTransform),
                "Front"));
        break;
      case SIM:
        gyro = new Gyro(new GyroIOSim());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOSim(
                    "Front", CameraConstants.frontCameraProperties, CameraConstants.frontTransform),
                "Front"));
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    robotOdometry =
        new RobotOdometry(driveSubsystem, gyro, aprilTagVisions.toArray(AprilTagVision[]::new));
    robotOdometry.addEstimator("Normal", RobotOdometry.getDefaultEstimator());
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem));
    DriveWeightCommand.addPersistentWeight(
        new JoystickDriveWeight(
            () -> driveController.getLeftY(),
            () -> driveController.getLeftX(),
            () -> driveController.getRightX(),
            gyro));

    driveController.start().onTrue(gyro.resetGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
