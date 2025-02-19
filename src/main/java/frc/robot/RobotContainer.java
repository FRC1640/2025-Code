// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.Mode;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.constants.RobotConstants.RobotConfigConstants;
import frc.robot.constants.RobotConstants.RobotDimensions;
import frc.robot.constants.RobotConstants.TestConfig;
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
import frc.robot.subsystems.climber.commands.ClimberRoutines;
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
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSim;
import frc.robot.subsystems.winch.WinchIOSparkMax;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.controller.PresetBoard;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.dashboard.PIDInfo.PIDCommandRegistry;
import frc.robot.util.logging.LogRunner;
import frc.robot.util.testModeControls.TestModeController;
import frc.robot.util.tools.AllianceManager;
import frc.robot.util.tools.DistanceManager;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
  private final WinchSubsystem winchSubsystem;
  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final PresetBoard presetBoard = new PresetBoard(2);
  private final PresetBoard simBoard = new PresetBoard(3);
  private final PresetBoard testBoard = new PresetBoard(4);
  private final PresetBoard motorBoard = new PresetBoard(5);

  private final AlertsManager alertsManager;

  // Dashboard
  private final Dashboard dashboard;

  private final ReefDetector reefDetector;

  private final LogRunner logRunner;
  private final GantryCommandFactory gantryCommandFactory;
  private final LiftCommandFactory liftCommandFactory;
  private final CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private final DriveCommandFactory driveCommandFactory;
  private final ClimberCommandFactory climberCommandFactory;
  private final ClimberRoutines climberRoutines;
  private final AutoScoringCommandFactory autoScoringCommandFactory;
  private final AlgaeCommandFactory algaeCommandFactory;

  CoralPreset presetActive = CoralPreset.Safe;

  private FollowPathNearest followPathNearest;

  private final JoystickDriveWeight joystickDriveWeight;

  private CoralPreset coralPreset = CoralPreset.Safe;
  private boolean algaeMode = false;
  private boolean gantryAuto = false;

  public RobotContainer() {

    switch (Robot.getMode()) {
      case REAL:
        gyro = new Gyro(new GyroIONavX());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCameraLeft),
                CameraConstants.frontCameraLeft));

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCameraRight),
                CameraConstants.frontCameraRight));
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

        winchSubsystem =
            new WinchSubsystem(
                RobotConfigConstants.climberSubsystemEnabled
                    ? new WinchIOSparkMax()
                    : new WinchIO() {});

        algaeIntakeSubsystem =
            new AlgaeSubsystem(
                RobotConfigConstants.algaeIntakeEnabled ? new AlgaeIOSpark() : new AlgaeIO() {});
        break;
      case SIM:
        gyro = new Gyro(RobotConfigConstants.gyroEnabled ? new GyroIOSim() : new GyroIO() {});

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOSim(
                    CameraConstants.frontCameraLeft,
                    () -> new Pose3d(RobotOdometry.instance.getPose("Main"))),
                CameraConstants.frontCameraLeft));
        reefDetector =
            new ReefDetector(
                RobotConfigConstants.reefDetectorEnabled
                    ? new ReefDetectorIOSim(() -> 0.0, () -> 0.0)
                    : new ReefDetectorIO() {});
        gantrySubsystem =
            new GantrySubsystem(
                RobotConfigConstants.gantrySubsystemEnabled
                    ? new GantryIOSim(() -> simBoard.getLl2())
                    : new GantryIO() {});
        liftSubsystem =
            new LiftSubsystem(
                RobotConfigConstants.liftSubsystemEnabled
                    ? new LiftIOSim(() -> simBoard.getLl3())
                    : new LiftIO() {});
        coralOuttakeSubsystem =
            new CoralOuttakeSubsystem(
                RobotConfigConstants.coralOuttakeSubsystemEnabled
                    ? new CoralOuttakeIOSim(() -> simBoard.getRl2())
                    : new CoralOuttakeIO() {});
        climberSubsystem =
            new ClimberSubsystem(
                RobotConfigConstants.climberSubsystemEnabled
                    ? new ClimberIOSim(() -> simBoard.getRl4())
                    : new ClimberIO() {});
        winchSubsystem =
            new WinchSubsystem(
                RobotConfigConstants.climberSubsystemEnabled ? new WinchIOSim() : new WinchIO() {});
        algaeIntakeSubsystem =
            new AlgaeSubsystem(
                RobotConfigConstants.algaeIntakeEnabled
                    ? new AlgaeIOSim(() -> simBoard.getLl4())
                    : new AlgaeIO() {});
        break;
      default:
        gyro = new Gyro(new GyroIO() {});
        reefDetector = new ReefDetector(new ReefDetectorIO() {});
        gantrySubsystem = new GantrySubsystem(new GantryIO() {});
        liftSubsystem = new LiftSubsystem(new LiftIO() {});
        coralOuttakeSubsystem = new CoralOuttakeSubsystem(new CoralOuttakeIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        winchSubsystem = new WinchSubsystem(new WinchIO() {});
        algaeIntakeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
        break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    gantryCommandFactory = new GantryCommandFactory(gantrySubsystem, reefDetector);
    liftCommandFactory = new LiftCommandFactory(liftSubsystem);
    coralOuttakeCommandFactory = new CoralOuttakeCommandFactory(coralOuttakeSubsystem);
    driveCommandFactory = new DriveCommandFactory(driveSubsystem);
    climberCommandFactory = new ClimberCommandFactory(climberSubsystem, winchSubsystem);
    climberRoutines = new ClimberRoutines(climberCommandFactory);
    algaeCommandFactory = new AlgaeCommandFactory(algaeIntakeSubsystem);
    logRunner = new LogRunner();
    autoScoringCommandFactory =
        new AutoScoringCommandFactory(
            gantryCommandFactory,
            liftCommandFactory,
            liftSubsystem,
            coralOuttakeCommandFactory,
            coralOuttakeSubsystem,
            algaeCommandFactory,
            algaeIntakeSubsystem);
    setDefaultCommands();
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro, visionArray);
    dashboard = new Dashboard(driveSubsystem, liftSubsystem, gantrySubsystem, driveController);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(
        () -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.",
        AlertType.kWarning);
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

    // liftSubsystem.setDefaultCommand(
    //     liftCommandFactory.liftApplyVoltageCommand(() -> -4 * operatorController.getRightY()));

    generateNamedCommands();
    configureBindings();
    if (TestConfig.reconstructTrigger) {
      TestModeController.reconstructTrigger = () -> (testBoard.getRawAxis(2) == 1);
    }
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

  public void mapPIDtoCommand() {
    PIDCommandRegistry.attachPIDCommand(
        "gantryPID", (x) -> gantryCommandFactory.gantryPIDCommand(() -> x));
    PIDCommandRegistry.attachPIDCommand(
        "gantryVelocityPID", (x) -> gantryCommandFactory.gantrySetVelocityCommand(() -> x));
    PIDCommandRegistry.attachPIDCommand(
        "climberLiftPID", (x) -> climberCommandFactory.setElevatorPosPID(() -> x));
    PIDCommandRegistry.attachPIDCommand(
        "winchPID", (x) -> climberCommandFactory.setWinchPosPID(() -> x));
    PIDCommandRegistry.attachProfiledPIDCommand(
        "LiftPPID", (x) -> liftCommandFactory.runLiftMotionProfile(() -> x));
  }

  public void reconstructKeybinds() {
    generateNamedCommands();
    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    generateNamedCommands();
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveCommandFactory));
    algaeIntakeSubsystem.setDefaultCommand(algaeCommandFactory.algaePassiveCommand());
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
        .onTrue(setupAutoPlace(() -> coralPreset));
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
        .whileTrue(
            algaeCommandFactory
                .processCommand()
                .beforeStarting(() -> joystickDriveWeight.setEnabled(false))
                .finallyDo(() -> joystickDriveWeight.setEnabled(true)));
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

    driveController
        .povDown()
        .onTrue(new InstantCommand(() -> joystickDriveWeight.setEnabled(true)));

    driveController.b().whileTrue(coralOuttakeCommandFactory.setIntakeVoltage(() -> 12));
    // rumble
    new Trigger(() -> coralOuttakeSubsystem.hasCoral() /* driveController.getHID().getXButton() */)
        .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kLeftRumble, 1)))
        .onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kLeftRumble, 0)));
    new Trigger(() -> algaeIntakeSubsystem.hasAlgae() /* driveController.getHID().getYButton() */)
        .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kRightRumble, 0.5)))
        .onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kRightRumble, 0)));
    // new Trigger(() -> coralPreset.isRight())

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
    new Trigger(() -> presetBoard.getTrough())
        .onTrue(new InstantCommand(() -> coralPreset = CoralPreset.Trough));
    // lift/gantry manual controls
    operatorController.start().whileTrue(liftCommandFactory.liftHomeCommand());
    operatorController.a().onTrue(setupAutoPlace(() -> coralPreset));

    new Trigger(() -> (!coralOuttakeSubsystem.isCoralDetected())).onTrue(runLiftToSafe());
    new Trigger(
            () ->
                algaeIntakeSubsystem.hasAlgae()
                    && RobotOdometry.instance
                            .getPose("Main")
                            .getTranslation()
                            .getDistance(getTarget().getTranslation())
                        > 0.3)
        .onTrue(runLiftToSafe());
    operatorController.b().onTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 0).repeatedly());
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
    // motor board
    new Trigger(() -> motorBoard.getLl2())
        .onTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 1));
    new Trigger(() -> motorBoard.getLl3())
        .onTrue(gantryCommandFactory.gantryApplyVoltageCommand(() -> 1));
    new Trigger(() -> motorBoard.getLl4())
        .onTrue(coralOuttakeCommandFactory.setIntakeVoltage(() -> 1));
    new Trigger(() -> motorBoard.getRl4())
        .onTrue(algaeCommandFactory.setMotorVoltages(() -> 1, () -> 1));
    new Trigger(() -> motorBoard.getRl3())
        .onTrue(climberCommandFactory.elevatorApplyVoltageCommand(() -> 1));
    new Trigger(() -> motorBoard.getRl2())
        .onTrue(climberCommandFactory.winchApplyVoltageCommand(() -> 1));

    // climber button bindings:
    operatorController.povUp().toggleOnTrue(climberRoutines.initiatePart1());
    operatorController.povDown().toggleOnTrue(climberRoutines.initiatePart2());
    operatorController.povLeft().toggleOnTrue(climberRoutines.resetClimber());
    operatorController.povRight().whileTrue(climberCommandFactory.liftHomeCommand());
    mapPIDtoCommand();
  }

  public Command getAutonomousCommand() {
    return homing().andThen(dashboard.getAutoChooserCommand());
  }

  public Command homing() {
    return new ConditionalCommand(
        new InstantCommand(),
        gantryCommandFactory
            .gantryHomeCommand()
            .alongWith(liftCommandFactory.liftHomeCommand())
            .alongWith(climberCommandFactory.liftHomeCommand()),
        () -> Robot.getMode() == Mode.SIM);
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

  public Command setupAutoPlace(Supplier<CoralPreset> coralPreset) {
    return new InstantCommand(
            () -> {
              presetActive = coralPreset.get();
            })
        .andThen(
            liftCommandFactory
                .runLiftMotionProfile(
                    () -> algaeMode ? presetActive.getLift() : presetActive.getLiftAlgae())
                .repeatedly())
        .alongWith(
            autoScoringCommandFactory.gantryAlignCommand(
                () -> presetActive, () -> AllianceManager.onDsSideReef(() -> getTarget())));
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
    return liftCommandFactory
        .runLiftMotionProfile(() -> CoralPreset.Safe.getLift())
        .repeatedly()
        .alongWith(
            gantryCommandFactory.gantryPIDCommand(() -> GantryConstants.gantryLimits.low / 2));
  }

  public void generateNamedCommands() {
    NamedCommands.registerCommand(
        "EnableAprilTags", new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(true)));
    NamedCommands.registerCommand(
        "DisableAprilTags",
        new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(false)));
    NamedCommands.registerCommand(
        "DisableRotation", new InstantCommand(() -> PathplannerWeight.overrideRotation(() -> 0)));
    NamedCommands.registerCommand(
        "NormalRotation", new InstantCommand(() -> PathplannerWeight.clearRotationOverride()));

    NamedCommands.registerCommand("SetModeAlgae", new InstantCommand(() -> algaeMode = true));
    NamedCommands.registerCommand("SetModeCoral", new InstantCommand(() -> algaeMode = false));
    NamedCommands.registerCommand("SetGantryRight", new InstantCommand(() -> gantryAuto = true));
    NamedCommands.registerCommand("SetGantryLeft", new InstantCommand(() -> gantryAuto = false));

    NamedCommands.registerCommand("SetupSafe", setupAutoPlace(() -> CoralPreset.Safe));

    NamedCommands.registerCommand("PlaceTrough", autoScoringCommandFactory.placeTrough());

    NamedCommands.registerCommand("StartSetup", setupAutoPlace(() -> coralPreset));

    NamedCommands.registerCommand(
        "logtest", new InstantCommand(() -> Logger.recordOutput("logtest", true)));

    NamedCommands.registerCommand(
        "SetupL4",
        new InstantCommand(
            () -> coralPreset = gantryAuto ? CoralPreset.RightL4 : CoralPreset.LeftL4));
    NamedCommands.registerCommand(
        "SetupL3",
        new InstantCommand(
            () -> coralPreset = gantryAuto ? CoralPreset.RightL3 : CoralPreset.LeftL3));
    NamedCommands.registerCommand(
        "SetupL2",
        new InstantCommand(
            () -> coralPreset = gantryAuto ? CoralPreset.RightL2 : CoralPreset.LeftL2));

    NamedCommands.registerCommand("AutoReef", getPlaceCommand());

    NamedCommands.registerCommand(
        "HaveCoral", new InstantCommand(() -> coralOuttakeSubsystem.setHasCoral(true)));

    NamedCommands.registerCommand(
        "WaitForAlign",
        new WaitUntilCommand(
            () ->
                liftSubsystem.isAtPreset(
                        algaeMode ? coralPreset.getLift() : coralPreset.getLiftAlgae())
                    && (gantrySubsystem.isAtPreset(coralPreset, true) || algaeMode)
                    && followPathNearest.isAutoalignComplete()));

    NamedCommands.registerCommand(
        "WaitForLift",
        new WaitUntilCommand(
                () ->
                    liftSubsystem.isAtPreset(
                            algaeMode ? coralPreset.getLift() : coralPreset.getLiftAlgae())
                        && (gantrySubsystem.isAtPreset(coralPreset, true) || algaeMode))
            .deadlineFor(setupAutoPlace(() -> coralPreset)));

    NamedCommands.registerCommand(
        "AutoAlign",
        new InstantCommand(() -> followPathNearest.startPath())
            .alongWith(
                new WaitUntilCommand(() -> followPathNearest.isAutoalignComplete())
                    .finallyDo(() -> followPathNearest.stopPath())));

    NamedCommands.registerCommand(
        "WaitForCoral", new WaitUntilCommand(() -> coralOuttakeSubsystem.hasCoral()));

    NamedCommands.registerCommand("Process", algaeCommandFactory.processCommand());
  }
}
