// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.constants.RobotConstants.RobotConfigConstants;
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
import frc.robot.sensors.reefdetector.ReefDetectorIOLaserCAN;
import frc.robot.sensors.reefdetector.ReefDetectorIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.ClimberCommandFactory;
import frc.robot.subsystems.coralouttake.CoralOuttakeIO;
import frc.robot.subsystems.coralouttake.CoralOuttakeIOSim;
import frc.robot.subsystems.coralouttake.CoralOuttakeIOSparkMax;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveCommandFactory;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.DriveToNearestWeight;
import frc.robot.subsystems.drive.weights.DriveToPointWeight;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.gantry.GantryIO;
import frc.robot.subsystems.gantry.GantryIOSparkMax;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.LiftIO;
import frc.robot.subsystems.lift.LiftIOSim;
import frc.robot.subsystems.lift.LiftIOSpark;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.tools.AllianceManager;
import java.util.ArrayList;
import java.util.HashMap;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem; // drive, steer
  private final Gyro gyro;
  private final RobotOdometry robotOdometry;
  private final GantrySubsystem gantrySubsystem;
  private final LiftSubsystem liftSubsystem;
  private final CoralOuttakeSubsystem coralOuttakeSubsystem;
  private final ClimberSubsystem climberSubsystem; // lift, winch
  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final AlertsManager alertsManager;

  // Dashboard
  private final Dashboard dashboard;
  private HashMap<String, PIDController> pidKeys;

  private final ReefDetector reefDetector;

  private final GantryCommandFactory gantryCommandFactory;
  private final LiftCommandFactory liftCommandFactory;
  private final CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private final DriveCommandFactory driveCommandFactory;
  private final ClimberCommandFactory climberCommandFactory;

  public RobotContainer() {
    pidKeys.put("test", new PIDController(0, 0, 0));
    switch (Robot.getMode()) {
      case REAL:
        gyro = new Gyro(new GyroIONavX());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCamera),
                CameraConstants.frontCamera));
        reefDetector =
            new ReefDetector(
                RobotConfigConstants.reefDetectorEnabled
                    ? new ReefDetectorIOLaserCAN()
                    : new ReefDetectorIO() {});
        gantrySubsystem =
            new GantrySubsystem(
                RobotConfigConstants.gantrySubsystemEnabled
                    ? new GantryIOSparkMax()
                    : new GantryIO() {});
        liftSubsystem =
            new LiftSubsystem(
                RobotConfigConstants.liftSubsystemEnabled ? new LiftIOSpark() : new LiftIO() {});

        coralOuttakeSubsystem =
            new CoralOuttakeSubsystem(
                RobotConfigConstants.coralOuttakeSubsystemEnabled
                    ? new CoralOuttakeIOSparkMax()
                    : new CoralOuttakeIO() {});
        climberSubsystem =
            new ClimberSubsystem(
                RobotConfigConstants.climberSubsystemEnabled
                    ? new ClimberIOSparkMax()
                    : new ClimberIO() {});

        break;
      case SIM:
        gyro = new Gyro(RobotConfigConstants.gyroEnabled ? new GyroIOSim() : new GyroIO() {});

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOSim(
                    CameraConstants.frontCamera,
                    () -> new Pose3d(RobotOdometry.instance.getPose("Main"))),
                CameraConstants.frontCamera));
        reefDetector =
            new ReefDetector(
                RobotConfigConstants.reefDetectorEnabled
                    ? new ReefDetectorIOSim(() -> 0.0, () -> 0.0)
                    : new ReefDetectorIO() {});
        gantrySubsystem =
            new GantrySubsystem(
                RobotConfigConstants.gantrySubsystemEnabled
                    ? new GantryIOSparkMax()
                    : new GantryIO() {});
        liftSubsystem =
            new LiftSubsystem(
                RobotConfigConstants.liftSubsystemEnabled ? new LiftIOSim() : new LiftIO() {});
        coralOuttakeSubsystem =
            new CoralOuttakeSubsystem(
                RobotConfigConstants.coralOuttakeSubsystemEnabled
                    ? new CoralOuttakeIOSim(() -> true)
                    : new CoralOuttakeIO() {});
        climberSubsystem =
            new ClimberSubsystem(
                RobotConfigConstants.climberSubsystemEnabled
                    ? new ClimberIOSim()
                    : new ClimberIO() {});
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        reefDetector = new ReefDetector(new ReefDetectorIO() {});
        gantrySubsystem = new GantrySubsystem(new GantryIO() {});
        liftSubsystem = new LiftSubsystem(new LiftIO() {});
        coralOuttakeSubsystem = new CoralOuttakeSubsystem(new CoralOuttakeIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro, visionArray);
    dashboard = new Dashboard(driveSubsystem, liftSubsystem, driveController);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(
        () -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.",
        AlertType.kWarning);

    gantryCommandFactory = new GantryCommandFactory(gantrySubsystem, reefDetector);
    liftCommandFactory = new LiftCommandFactory(liftSubsystem);
    coralOuttakeCommandFactory = new CoralOuttakeCommandFactory(coralOuttakeSubsystem);
    driveCommandFactory = new DriveCommandFactory(driveSubsystem);
    climberCommandFactory = new ClimberCommandFactory(climberSubsystem);

    // set defaults

    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveCommandFactory));
    configureBindings();
  }

  private void configureBindings() {
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
                AllianceManager.chooseFromAlliance(
                    FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed),
            gyro,
            (x) -> RobotConstants.addRobotDim(x)),
        driveController.a());

    DriveWeightCommand.createWeightTrigger(
        new DriveToPointWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                AllianceManager.chooseFromAlliance(
                    FieldConstants.processorPositionBlue, FieldConstants.processorPositionRed),
            gyro),
        driveController.x());

    driveController.start().onTrue(gyro.resetGyroCommand());

    // gantry button bindings:

    operatorController.x().whileTrue(gantryCommandFactory.gantrySweep(true));
    operatorController.b().whileTrue(gantryCommandFactory.gantrySweep(false));
    operatorController
        .rightBumper()
        .whileTrue(gantryCommandFactory.gantryApplyVoltageCommand(() -> 2));

    operatorController
        .leftBumper()
        .whileTrue(gantryCommandFactory.gantryApplyVoltageCommand(() -> -2));

    operatorController.back().onTrue(new InstantCommand(() -> gantrySubsystem.resetEncoder()));

    operatorController.a().whileTrue(liftCommandFactory.runLiftMotionProfile(() -> 1.0));

    // intake button bindings:
    coralOuttakeCommandFactory.constructTriggers();
    operatorController
        .rightTrigger()
        .whileTrue(
            coralOuttakeCommandFactory.setIntakeVoltage(
                () -> CoralOuttakeConstants.passiveSpeed * 12));
  }

  public Command getAutonomousCommand() {
    return dashboard
        .getAutoChooserCommand()
        .andThen(driveCommandFactory.runVelocityCommand(() -> new ChassisSpeeds()));
  }

  public Command getSysidCommand() {
    return dashboard.getSysidCommand();
  }
}
