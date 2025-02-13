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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.constants.RobotConstants.CameraConstants;
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
import frc.robot.subsystems.algae.commands.AlgaeCommandFactory;
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
import frc.robot.subsystems.drive.weights.FollowPathNearest;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.drive.weights.PathplannerWeight;
import frc.robot.subsystems.drive.weights.RotateToAngleWeight;
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
  private final AlgaeCommandFactory algaeCommandFactory;

  private FollowPathNearest followPathNearest;

  private final JoystickDriveWeight joystickDriveWeight;

  private CoralPreset coralPreset = CoralPreset.Safe;
  private boolean algaeMode = false;

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
    algaeCommandFactory = new AlgaeCommandFactory(algaeIntakeSubsystem);

    autoScoringCommandFactory =
        new AutoScoringCommandFactory(
            gantryCommandFactory,
            liftCommandFactory,
            liftSubsystem,
            coralOuttakeCommandFactory,
            coralOuttakeSubsystem,
            algaeCommandFactory,
            algaeIntakeSubsystem);

    // set defaults

    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveCommandFactory));
    algaeIntakeSubsystem.setDefaultCommand(algaeCommandFactory.algaePassiveCommand());
    // weights
    joystickDriveWeight =
        new JoystickDriveWeight(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper(),
            driveController.leftTrigger());
    followPathNearest =
        new FollowPathNearest(
            () -> RobotOdometry.instance.getPose("Main"),
            gyro,
            () -> chooseAlignPos(),
            AutoAlignConfig.pathConstraints,
            (x) ->
                coralAdjust(
                    DistanceManager.addRotatedDim(
                        x, RobotDimensions.robotLength / 2, x.getRotation()),
                    () -> coralPreset),
            driveSubsystem);

    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

    DriveWeightCommand.addPersistentWeight(new AntiTipWeight(gyro));

    DriveWeightCommand.addPersistentWeight(
        new PathplannerWeight(gyro, () -> RobotOdometry.instance.getPose("Main")));
    configureBindings();
  }

  public Pose2d coralAdjust(Pose2d pose, Supplier<CoralPreset> preset) {
    if (algaeMode) {
      return pose;
    }
    boolean alliance = AllianceManager.onDsSideReef(() -> getTarget());
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
    // lift/gantry presets for autoalign
    new Trigger(
            () ->
                RobotOdometry.instance
                            .getPose("Main")
                            .getTranslation()
                            .getDistance(getTarget().getTranslation())
                        < 1.5
                    && followPathNearest.isEnabled())
        .onTrue(setupAutoPlace());
    // coral place routine for autoalign
    // new Trigger(() -> coralAutoAlignWeight.isAutoalignComplete())
    //     .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kRightRumble, 1)));
    followPathNearest.generateTrigger(
        () -> driveController.a().getAsBoolean() && !followPathNearest.isAutoalignComplete());
    new Trigger(
            () ->
                followPathNearest.isAutoalignComplete()
                    && liftSubsystem.isAtPreset(
                        algaeMode ? coralPreset.getLift() : coralPreset.getLiftAlgae())
                    && (gantrySubsystem.isAtPreset(
                            coralPreset, AllianceManager.onDsSideReef(() -> getTarget()))
                        || algaeMode)
                    && !algaeIntakeSubsystem.hasAlgae())
        .onTrue(getAutoPlaceCommand());
    new Trigger(
            () ->
                followPathNearest.isAutoalignComplete()
                    && liftSubsystem.isAtPreset(CoralPreset.Safe.lift)
                    && algaeIntakeSubsystem.hasAlgae())
        .whileTrue(algaeCommandFactory.processCommand());
    new Trigger(() -> presetBoard.povIsUpwards())
        .onTrue(new InstantCommand(() -> algaeMode = true));

    new Trigger(() -> presetBoard.povIsDownwards())
        .onTrue(new InstantCommand(() -> algaeMode = false));

    new Trigger(
            () ->
                RobotOdometry.instance
                        .getPose("Main")
                        .getTranslation()
                        .getDistance(getTarget().getTranslation())
                    > 2)
        .onTrue(runLiftToSafe());

    DriveWeightCommand.createWeightTrigger(
        new RotateToAngleWeight(
            () -> RobotOdometry.instance.getPose("Main"),
            () ->
                DistanceManager.getNearestPosition(
                        RobotOdometry.instance.getPose("Main"),
                        AllianceManager.chooseFromAlliance(
                            FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed))
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(180))),
        driveController.leftBumper());

    driveController.b().onTrue(new InstantCommand(() -> joystickDriveWeight.setEnabled(true)));
    // reset gyro
    driveController.start().onTrue(gyro.resetGyroCommand());

    // gantry button bindings:
    operatorController.x().whileTrue(getAutoPlaceCommand());
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
    driveController.povUp().whileTrue(autoScoringCommandFactory.outtakeCoralCommand());
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
    operatorController.a().onTrue(setupAutoPlace());

    new Trigger(
            () ->
                (!coralOuttakeSubsystem.isCoralDetected()
                    || (algaeIntakeSubsystem.hasAlgae()
                        && !coralOuttakeSubsystem.isCoralDetected())))
        .onTrue(runLiftToSafe());
    operatorController
        .b()
        .onTrue(
            new InstantCommand(
                () ->
                    liftSubsystem.setDefaultCommand(
                        liftCommandFactory.liftApplyVoltageCommand(() -> 0))));
    operatorController.y().onTrue(runLiftToSafe());

    driveController
        .rightTrigger()
        .and(() -> !algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(true)
                .andThen(algaeCommandFactory.setMotorVoltages(() -> 5, () -> 5)))
        .onFalse(algaeCommandFactory.setSolenoidState(false));

    operatorController
        .rightTrigger()
        .and(() -> algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(true)
                .andThen(algaeCommandFactory.processCommand()));
  }

  public Command getAutonomousCommand() {
    return dashboard.getAutoChooserCommand().alongWith(gantryCommandFactory.gantryHomeCommand());
  }

  public Pose2d getTarget() {
    return followPathNearest.getFinalPosition();
  }

  public Command getAutoPlaceCommand() {
    return getPlaceCommand()
        .beforeStarting(new InstantCommand(() -> joystickDriveWeight.setEnabled(false)))
        .finallyDo(() -> joystickDriveWeight.setEnabled(true));
  }

  public Command getPlaceCommand() {
    return new ConditionalCommand(
        autoScoringCommandFactory.algaeAutoPickup(),
        autoScoringCommandFactory.autoPlace(),
        () -> algaeMode);
  }

  public Command setupAutoPlace() {
    return new InstantCommand(
            () ->
                liftSubsystem.setDefaultCommand(
                    liftCommandFactory.runLiftMotionProfile(
                        algaeMode ? coralPreset.getLift() : coralPreset.getLiftAlgae())))
        .alongWith(
            autoScoringCommandFactory.gantryAlignCommand(
                () -> coralPreset, () -> AllianceManager.onDsSideReef(() -> getTarget())));
  }

  public Pose2d[] chooseAlignPos() {
    return algaeIntakeSubsystem.hasAlgae()
        ? new Pose2d[] {
          AllianceManager.chooseFromAlliance(
              FieldConstants.processorPositionBlue, FieldConstants.processorPositionRed)
        }
        : AllianceManager.chooseFromAlliance(
            FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed);
  }

  public Command runLiftToSafe() {
    return new InstantCommand(
            () ->
                liftSubsystem.setDefaultCommand(
                    liftCommandFactory.runLiftMotionProfile(() -> CoralPreset.Safe.getLift())))
        .alongWith(
            gantryCommandFactory.gantryPIDCommand(() -> GantryConstants.gantryLimits.low / 2));
  }
}
