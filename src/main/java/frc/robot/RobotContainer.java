// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.constants.RobotConstants.RobotConfigConstants;
import frc.robot.constants.RobotConstants.RobotDimensions;
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
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeIOSpark;
import frc.robot.subsystems.algae.AlgaeSubsystem;
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
import frc.robot.subsystems.drive.commands.AutoScoringCommandFactory;
import frc.robot.subsystems.drive.commands.DriveCommandFactory;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.AntiTipWeight;
import frc.robot.subsystems.drive.weights.DriveToNearestWeight;
import frc.robot.subsystems.drive.weights.DriveToPointWeight;
import frc.robot.subsystems.drive.weights.FollowPathNearest;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.drive.weights.PathplannerWeight;
import frc.robot.subsystems.gantry.GantryIO;
import frc.robot.subsystems.gantry.GantryIOSim;
import frc.robot.subsystems.gantry.GantryIOSparkMax;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.LiftIO;
import frc.robot.subsystems.lift.LiftIOSim;
import frc.robot.subsystems.lift.LiftIOSpark;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.controller.PresetBoard;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.tools.AllianceManager;
import frc.robot.util.tools.DistanceManager;
import java.util.ArrayList;
import java.util.function.Supplier;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;
  private final RobotOdometry robotOdometry;
  private final GantrySubsystem gantrySubsystem;
  private final LiftSubsystem liftSubsystem;
  private final CoralOuttakeSubsystem coralOuttakeSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final AlgaeSubsystem algaeIntakeSubsystem;
  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final PresetBoard presetBoard = new PresetBoard(2);
  private final AlertsManager alertsManager;

  // Dashboard
  private final Dashboard dashboard;

  private final ReefDetector reefDetector;

  private final GantryCommandFactory gantryCommandFactory;
  private final LiftCommandFactory liftCommandFactory;
  private final CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private final DriveCommandFactory driveCommandFactory;
  private final ClimberCommandFactory climberCommandFactory;
  private final AutoScoringCommandFactory autoScoringCommandFactory;
  private CoralPreset coralPreset = CoralPreset.Safe;

  private DriveToNearestWeight coralAutoAlignWeight;

  public FollowPathNearest followPathNearest;

  public RobotContainer() {
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
        algaeIntakeSubsystem =
            new AlgaeSubsystem(
                RobotConfigConstants.algaeIntakeEnabled ? new AlgaeIOSpark() : new AlgaeIO() {});

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
                    ? new GantryIOSim(operatorController.y())
                    : new GantryIO() {});
        liftSubsystem =
            new LiftSubsystem(
                RobotConfigConstants.liftSubsystemEnabled ? new LiftIOSim() : new LiftIO() {});
        coralOuttakeSubsystem =
            new CoralOuttakeSubsystem(
                RobotConfigConstants.coralOuttakeSubsystemEnabled
                    ? new CoralOuttakeIOSim(operatorController.leftBumper())
                    : new CoralOuttakeIO() {});
        climberSubsystem =
            new ClimberSubsystem(
                RobotConfigConstants.climberSubsystemEnabled
                    ? new ClimberIOSim()
                    : new ClimberIO() {});
        algaeIntakeSubsystem =
            new AlgaeSubsystem(
                RobotConfigConstants.algaeIntakeEnabled ? new AlgaeIOSim() : new AlgaeIO() {});
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        reefDetector = new ReefDetector(new ReefDetectorIO() {});
        gantrySubsystem = new GantrySubsystem(new GantryIO() {});
        liftSubsystem = new LiftSubsystem(new LiftIO() {});
        coralOuttakeSubsystem = new CoralOuttakeSubsystem(new CoralOuttakeIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        algaeIntakeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro, visionArray);
    dashboard = new Dashboard(driveSubsystem, liftSubsystem, gantrySubsystem, driveController);
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
    autoScoringCommandFactory =
        new AutoScoringCommandFactory(
            gantryCommandFactory,
            liftCommandFactory,
            liftSubsystem,
            coralOuttakeCommandFactory,
            coralOuttakeSubsystem);

    // set defaults

    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveCommandFactory));

    // weights
    coralAutoAlignWeight =
        new DriveToNearestWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                AllianceManager.chooseFromAlliance(
                    FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed),
            gyro,
            (x) -> x,
            driveSubsystem);

    coralAutoAlignWeight.setPoseFunction(
        (x) ->
            coralAdjust(
                DistanceManager.addRotatedDim(x, RobotDimensions.robotLength / 2, x.getRotation()),
                () -> coralPreset));

    DriveWeightCommand.addPersistentWeight(
        new JoystickDriveWeight(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper(),
            driveController.leftTrigger()));

    DriveWeightCommand.addPersistentWeight(new AntiTipWeight(gyro));

    DriveWeightCommand.addPersistentWeight(
        new PathplannerWeight(gyro, () -> RobotOdometry.instance.getPose("Main")));
    configureBindings();
  }

  public Pose2d coralAdjust(Pose2d pose, Supplier<CoralPreset> preset) {
    boolean alliance = AllianceManager.onDsSideReef(() -> coralAutoAlignWeight.getTarget());
    double side;
    switch (preset.get().getGantrySetpoint(alliance)) {
      case LEFT:
        side = 0.1;
        break;
      case RIGHT:
        side = -0.1;
        break;
      case CENTER:
        side = 0;
        break;
      default:
        side = 0;
        break;
    }
    return DistanceManager.addRotatedDim(
        pose, side, pose.getRotation().plus(Rotation2d.fromDegrees(90)));
  }

  private void configureBindings() {
    followPathNearest =
        new FollowPathNearest(
            () -> RobotOdometry.instance.getPose("Main"),
            gyro,
            AllianceManager.chooseFromAlliance(
                FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed),
            1,
            1,
            1,
            1,
            1,
            () -> PathplannerWeight.setpoint,
            (x) -> x);
    followPathNearest.setPoseFunction(
        (x) ->
            coralAdjust(
                DistanceManager.addRotatedDim(x, RobotDimensions.robotLength / 2, x.getRotation()),
                () -> coralPreset));
    followPathNearest.generateTrigger(driveController.y());
    // bind reef align
    DriveWeightCommand.createWeightTrigger(coralAutoAlignWeight, driveController.a());
    // lift/gantry presets for autoalign
    new Trigger(
            () ->
                coralAutoAlignWeight.getTargetDistance() < 1.5
                    && DriveWeightCommand.checkWeight(coralAutoAlignWeight))
        .onTrue(
            new InstantCommand(
                    () ->
                        liftSubsystem.setDefaultCommand(
                            liftCommandFactory.runLiftMotionProfile(() -> coralPreset.getLift())))
                .alongWith(
                    autoScoringCommandFactory.gantryAlignCommand(
                        () -> coralPreset,
                        () ->
                            AllianceManager.onDsSideReef(() -> coralAutoAlignWeight.getTarget()))));
    // coral place routine for autoalign
    // new Trigger(() -> coralAutoAlignWeight.isAutoalignComplete())
    //     .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kRightRumble, 1)));
    new Trigger(
            () ->
                coralAutoAlignWeight.isAutoalignComplete()
                    // && liftSubsystem.isAtPreset(coralPreset)
                    && gantrySubsystem.isAtPreset(
                        coralPreset,
                        AllianceManager.onDsSideReef(() -> coralAutoAlignWeight.getTarget())))
        .onTrue(autoScoringCommandFactory.autoPlace());
    // processor autoalign
    DriveWeightCommand.createWeightTrigger(
        new DriveToPointWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                AllianceManager.chooseFromAlliance(
                    FieldConstants.processorPositionBlue, FieldConstants.processorPositionRed),
            gyro),
        driveController.x());
    // reset gyro
    driveController.start().onTrue(gyro.resetGyroCommand());
    // gantry button bindings:
    operatorController.x().whileTrue(gantryCommandFactory.gantryDriftCommand());
    operatorController
        .rightBumper()
        .whileTrue(gantryCommandFactory.gantrySetVelocityCommand(() -> GantryConstants.alignSpeed));
    operatorController
        .leftBumper()
        .whileTrue(
            gantryCommandFactory.gantrySetVelocityCommand(() -> -GantryConstants.alignSpeed));
    operatorController.back().whileTrue(gantryCommandFactory.gantryHomeCommand());
    // intake button bindings:
    coralOuttakeCommandFactory.constructTriggers();
    driveController
        .rightTrigger()
        .whileTrue(
            coralOuttakeCommandFactory.setIntakeVoltage(
                () -> CoralOuttakeConstants.passiveSpeed * 12));
    // preset board
    new Trigger(() -> presetBoard.getLl2())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.LeftL2));
    new Trigger(() -> presetBoard.getRl2())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.RightL2));
    new Trigger(() -> presetBoard.getLl3())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.LeftL3));
    new Trigger(() -> presetBoard.getRl3())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.RightL3));
    new Trigger(() -> presetBoard.getLl4())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.LeftL4));
    new Trigger(() -> presetBoard.getRl4())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.RightL4));
    new Trigger(() -> presetBoard.getTroph())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.Troph));
    // lift/gantry manual controls
    operatorController
        .a()
        .onTrue(
            new InstantCommand(
                    () ->
                        liftSubsystem.setDefaultCommand(
                            liftCommandFactory.runLiftMotionProfile(() -> coralPreset.getLift())))
                .alongWith(
                    autoScoringCommandFactory.gantryAlignCommand(
                        () -> coralPreset,
                        () ->
                            AllianceManager.onDsSideReef(
                                () -> coralAutoAlignWeight.getTarget())))); // TODO jitter?

    new Trigger(() -> coralOuttakeSubsystem.isCoralDetected())
        .onFalse(
            new InstantCommand(
                    () ->
                        liftSubsystem.setDefaultCommand(
                            liftCommandFactory.runLiftMotionProfile(
                                () -> CoralPreset.Safe.getLift())))
                .alongWith(
                    gantryCommandFactory.gantryPIDCommand(
                        () -> GantryConstants.gantryLimits.low / 2)));
    operatorController
        .b()
        .onTrue(
            new InstantCommand(
                () ->
                    liftSubsystem.setDefaultCommand(
                        liftCommandFactory.liftApplyVoltageCommand(() -> 0))));
    operatorController
        .y()
        .onTrue(
            new InstantCommand(
                    () ->
                        liftSubsystem.setDefaultCommand(
                            liftCommandFactory.runLiftMotionProfile(
                                () -> CoralPreset.Safe.getLift())))
                .alongWith(
                    gantryCommandFactory.gantryPIDCommand(
                        () -> GantryConstants.gantryLimits.low / 2)));
  }

  public Command getAutonomousCommand() {
    return dashboard.getAutoChooserCommand().alongWith(gantryCommandFactory.gantryHomeCommand());
  }
}
