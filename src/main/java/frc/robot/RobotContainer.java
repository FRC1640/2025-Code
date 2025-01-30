// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIOPhotonvision;
import frc.robot.sensors.apriltag.AprilTagVisionIOSim;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.gyro.GyroIONavX;
import frc.robot.sensors.gyro.GyroIOSim;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.sensors.reefdetector.ReefDetector;
import frc.robot.sensors.reefdetector.ReefDetectorIO;
import frc.robot.sensors.reefdetector.ReefDetectorIODistanceSensor;
import frc.robot.sensors.reefdetector.ReefDetectorIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveToNearestWeight;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.DriveToPointWeight;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.gantry.GantryIO;
import frc.robot.subsystems.gantry.GantryIOSim;
import frc.robot.subsystems.gantry.GantryIOSparkMax;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.intakeoutake.IntakeOutakeIOInputsAutoLogged;
import frc.robot.subsystems.intakeoutake.IntakeOutakeIOSim;
import frc.robot.subsystems.intakeoutake.IntakeOutakeIOSparkMax;
import frc.robot.subsystems.intakeoutake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftIO;
import frc.robot.subsystems.lift.LiftIOSim;
import frc.robot.subsystems.lift.LiftIOSpark;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.dashboard.Dashboard;
import java.util.ArrayList;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;
  private final RobotOdometry robotOdometry;
  private final GantrySubsystem gantrySubsystem;
  private final LiftSubsystem liftSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final AlertsManager alertsManager;

  // Dashboard
  private final Dashboard dashboard;

  private final ReefDetector reefDetector;

  public RobotContainer() {
    switch (Robot.getMode()) {
      case REAL:
        gyro = new Gyro(new GyroIONavX());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCamera),
                CameraConstants.frontCamera));

        reefDetector = new ReefDetector(new ReefDetectorIODistanceSensor(4));
        gantrySubsystem = new GantrySubsystem(new GantryIOSparkMax());
        liftSubsystem = new LiftSubsystem(new LiftIOSpark());
        intakeSubsystem = new IntakeSubsystem(new IntakeOutakeIOSparkMax());
        break;
      case SIM:
        gyro = new Gyro(new GyroIOSim());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOSim(
                    CameraConstants.frontCamera,
                    () -> new Pose3d(RobotOdometry.instance.getPose("Main"))),
                CameraConstants.frontCamera));
        reefDetector = new ReefDetector(new ReefDetectorIOSim(() -> 0.0, () -> 0.0));
        gantrySubsystem = new GantrySubsystem(new GantryIOSim());
        liftSubsystem = new LiftSubsystem(new LiftIOSim());
        intakeSubsystem = new IntakeSubsystem(new IntakeOutakeIOSim());
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        reefDetector = new ReefDetector(new ReefDetectorIO() {});
        gantrySubsystem = new GantrySubsystem(new GantryIO() {});
        liftSubsystem = new LiftSubsystem(new LiftIO() {});
        intakeSubsystem = new IntakeSubsystem(null)
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro, visionArray);
    dashboard = new Dashboard(driveSubsystem, driveController);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(
        () -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.",
        AlertType.kWarning);
    GantryCommandFactory gantryCommandFactory = new GantryCommandFactory(gantrySubsystem);
    LiftCommandFactory liftCommandFactory = new LiftCommandFactory(liftSubsystem);
    gantrySubsystem.setDefaultCommand(
        gantryCommandFactory.gantryApplyVoltageCommand(() -> operatorController.getLeftX() * 6));
    liftSubsystem.setDefaultCommand(
        liftCommandFactory.liftApplyVoltageCommand(() -> operatorController.getRightY() * 6));
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem));
    DriveWeightCommand.addPersistentWeight(
        new JoystickDriveWeight(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper(),
            driveController.leftTrigger()));

    DriveWeightCommand.createWeightTrigger(
        new DriveToNearestWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                chooseFromAlliance(
                    FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed),
            gyro,
            (x) -> RobotConstants.addRobotDim(x)),
        driveController.a());

    DriveWeightCommand.createWeightTrigger(
        new DriveToPointWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                chooseFromAlliance(
                    FieldConstants.processorPositionBlue, FieldConstants.processorPositionRed),
            gyro),
        driveController.x());

    driveController.start().onTrue(gyro.resetGyroCommand());
  }

  public <T> T chooseFromAlliance(T valueBlue, T valueRed) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? valueRed : valueBlue;
  }

  public Command getAutonomousCommand() {
    return dashboard
        .getAutoChooserCommand()
        .andThen(driveSubsystem.runVelocityCommand(() -> new ChassisSpeeds()));
  }
}
