// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.gyro.GyroIONavX;
import frc.robot.sensors.gyro.GyroIOSim;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.util.dashboard.DashboardInit;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;
  private final RobotOdometry robotOdometry;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  public RobotContainer() {
    switch (Robot.getMode()) {
      case REAL:
        gyro = new Gyro(new GyroIONavX());
        break;
      case SIM:
        gyro = new Gyro(new GyroIOSim());
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro);
    robotOdometry.addEstimator("Normal", RobotOdometry.getDefaultEstimator());
    configureBindings();
    DashboardInit.testTestInit();
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
