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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.Mode;
import frc.robot.Robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.DriveConstants;
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
import frc.robot.subsystems.drive.weights.DynamicAlignWeight;
import frc.robot.subsystems.drive.weights.FollowPathDirect;
import frc.robot.subsystems.drive.weights.FollowPathNearest;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.drive.weights.LocalTagAlignWeight;
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
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSim;
import frc.robot.subsystems.winch.WinchIOSparkMax;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.ConfigEnums.TestMode.TestingSetting;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.controller.PresetBoard;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.dashboard.PIDInfo.PIDCommandRegistry;
import frc.robot.util.logging.LogRunner;
import frc.robot.util.misc.AllianceManager;
import frc.robot.util.misc.DistanceManager;
import frc.robot.util.periodic.PeriodicBase;
import frc.robot.util.periodic.PeriodicScheduler;
import frc.robot.util.pi.CoProcessInput;
import frc.robot.util.pi.OrangePILogger;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  private DriveSubsystem driveSubsystem;
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
  private final CommandXboxController pitController = new CommandXboxController(4);

  private final XboxController driveHID = driveController.getHID();
  private final XboxController opHID = operatorController.getHID();

  private final XboxController operatorControllerHID = operatorController.getHID();
  private final PresetBoard presetBoard = new PresetBoard(2);
  private final PresetBoard simBoard = new PresetBoard(3);
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
  double presetActive = 0;
  CoralPreset gantryPresetActive = CoralPreset.Safe;

  private boolean premoveLift = false;

  private FollowPathNearest followPathReef;
  private FollowPathDirect followPathCoral;
  private LocalTagAlignWeight localAlign;
  private DynamicAlignWeight dynamicAlign;

  private final JoystickDriveWeight joystickDriveWeight;

  private CoralPreset coralPreset = CoralPreset.Safe;
  private boolean algaeMode = false;
  private boolean gantryAuto = false;

  private boolean autoRampPos = false;

  private boolean isFC = true;

  boolean homed = false;

  public RobotContainer() {

    switch (Robot.getMode()) {
      case REAL:
        new CoProcessInput(new OrangePILogger());
        gyro = new Gyro(new GyroIONavX());
        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCameraRight),
                CameraConstants.frontCameraRight));

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCameraLeft),
                CameraConstants.frontCameraLeft));

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOPhotonvision(CameraConstants.frontCameraCenter),
                CameraConstants.frontCameraCenter));
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
        gyro =
            new Gyro(
                RobotConfigConstants.gyroEnabled
                    ? new GyroIOSim(
                        () ->
                            DriveConstants.kinematics.toChassisSpeeds(
                                    driveSubsystem.getActualSwerveStates())
                                .omegaRadiansPerSecond)
                    : new GyroIO() {});

        aprilTagVisions.add(
            new AprilTagVision(
                new AprilTagVisionIOSim(
                    CameraConstants.frontCameraLeft,
                    () -> new Pose3d(RobotOdometry.instance.getPose("Main"))),
                CameraConstants.frontCameraLeft));
        reefDetector =
            new ReefDetector(
                RobotConfigConstants.reefDetectorEnabled
                    ? new ReefDetectorIOSim(() -> presetBoard.getTrough())
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
    gantryCommandFactory = new GantryCommandFactory(gantrySubsystem, reefDetector);
    liftCommandFactory = new LiftCommandFactory(liftSubsystem);
    coralOuttakeCommandFactory = new CoralOuttakeCommandFactory(coralOuttakeSubsystem);
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
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);
    driveSubsystem = new DriveSubsystem(gyro);
    driveCommandFactory = new DriveCommandFactory(driveSubsystem);
    robotOdometry = new RobotOdometry(driveSubsystem, gyro, visionArray);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(
        () -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.",
        AlertType.kWarning);
    // weights
    // Will change later if we don't reset robot code before the match
    // Otherwise we wouldn't have a drive controller during the match
    joystickDriveWeight =
        new JoystickDriveWeight(
            () -> -driveController.getLeftY() - 1 * pitController.getLeftY(),
            () -> -driveController.getLeftX() - 1 * pitController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper(),
            driveController.leftTrigger(),
            () -> isFC,
            gyro,
            () -> liftSubsystem.getMotorPosition() > 0.2);

    followPathReef =
        new FollowPathNearest(
            () -> RobotOdometry.instance.getPose("Main"),
            gyro,
            () -> chooseAlignPos(),
            AutoAlignConfig.pathConstraints,
            (x) ->
                coralAdjust(
                    DistanceManager.addRotatedDim(
                        x, (algaeMode ? 0 : RobotDimensions.robotLength / 2), x.getRotation()),
                    () -> coralPreset),
            driveSubsystem);

    followPathCoral =
        new FollowPathDirect(
            () ->
                AllianceManager.chooseFromAlliance(
                    FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed),
            (x) ->
                DistanceManager.addRotatedDim(
                    x, ((-RobotDimensions.robotLength - 0.08) / 2), x.getRotation()),
            gyro,
            () -> RobotOdometry.instance.getPose("Main"),
            AutoAlignConfig.coralStationPathConstraints,
            driveSubsystem);

    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

    DriveWeightCommand.addPersistentWeight(new AntiTipWeight(gyro));

    DriveWeightCommand.addPersistentWeight(
        new PathplannerWeight(gyro, () -> RobotOdometry.instance.getPose("Main")));

    // liftSubsystem.setDefaultCommand(
    //     liftCommandFactory.liftApplyVoltageCommand(() -> -4 * operatorController.getRightY()));

    new Trigger(() -> Robot.getState() == RobotState.TELEOP && !homed).onTrue(homing());

    winchSubsystem.setDefaultCommand(
        climberCommandFactory.setWinchPosPID(() -> 348).onlyIf(() -> autoRampPos).repeatedly());

    climberSubsystem.setDefaultCommand(
        climberCommandFactory.setElevatorPosPID(() -> -5.8).onlyIf(() -> autoRampPos).repeatedly());

    algaeIntakeSubsystem.setDefaultCommand(
        algaeCommandFactory
            .setSolenoidState(() -> false)
            .onlyIf(() -> !algaeIntakeSubsystem.hasAlgae()));
    driveSubsystem.setDefaultCommand(
        DriveWeightCommand.create(
            driveCommandFactory, () -> liftSubsystem.getMotorPosition() > 0.3));

    localAlign =
        new LocalTagAlignWeight(
            () ->
                DistanceManager.getNearestPosition(
                    RobotOdometry.instance.getPose("Main"),
                    AllianceManager.chooseFromAlliance(
                        FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed)),
            () -> RobotOdometry.instance.getPose("Main").getRotation(),
            driveSubsystem,
            driveCommandFactory,
            gyro,
            Robot.getMode() == Mode.SIM ? visionArray : new AprilTagVision[] {visionArray[2]});

    dynamicAlign = new DynamicAlignWeight(followPathReef, localAlign);

    // winchSubsystem.setDefaultCommand(
    //     climberCommandFactory.winchApplyVoltageCommand(() -> -operatorController.getLeftY() *
    // 4));

    // climberSubsystem.setDefaultCommand(
    //     climberCommandFactory.elevatorApplyVoltageCommand(
    //         () -> -operatorController.getRightY() * 4));
    configureBindings();
    PeriodicScheduler.getInstance()
        .addPeriodic(
            new PeriodicBase() {
              @Override
              public void periodic() {
                Logger.recordOutput("AlgaeMode", algaeMode);
                Logger.recordOutput("CoralPreset", coralPreset);
                Logger.recordOutput("TargetPosAutoalign", getTarget());
                Logger.recordOutput("AutoAlignDone", dynamicAlign.isAutoalignComplete());
                Logger.recordOutput("LocalTagAlign/alignStage", dynamicAlign.getStage());
                Logger.recordOutput("LiftDone", liftSubsystem.isAtPreset(presetActive));
                Logger.recordOutput(
                    "LiftDoneAuto", liftSubsystem.isAtPreset(coralPreset.getLift()));
                Logger.recordOutput(
                    "GantryDone",
                    gantrySubsystem.isAtPreset(gantryPresetActive, true) || algaeMode);

                Logger.recordOutput("autoramppos", autoRampPos);

                Logger.recordOutput("WeightSize", DriveWeightCommand.getWeightsSize());

                Logger.recordOutput(
                    "DistFromTarget",
                    RobotOdometry.instance
                        .getPose("Main")
                        .getTranslation()
                        .getDistance(getTarget().getTranslation()));
                Logger.recordOutput("target", getTarget());
              }
            });

    generateNamedCommands();
    driveSubsystem.configurePathplanner();
    dashboard =
        new Dashboard(
            driveSubsystem,
            liftSubsystem,
            gantrySubsystem,
            climberSubsystem,
            algaeIntakeSubsystem,
            coralOuttakeSubsystem,
            winchSubsystem,
            driveController);
  }

  public Pose2d coralAdjust(Pose2d pose, Supplier<CoralPreset> preset) {
    if (algaeMode) {
      return pose;
    }
    boolean alliance = true;
    double side;
    switch (preset.get().getGantrySetpoint(alliance)) {
      case LEFT:
        side = 0;
        break;
      case RIGHT:
        side = 0;
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
    PIDCommandRegistry.attachProfiledPIDCommand(
        "GantryPPID", (x) -> gantryCommandFactory.runGantryMotionProfile(() -> x));
    PIDCommandRegistry.attachProfiledPIDCommand(
        "GantryVelPPID", (x) -> gantryCommandFactory.runGantryVelocityMotionProfile(() -> x));
  }

  private void configureBindings() {

    driveController.y().onTrue(new InstantCommand(() -> isFC = !isFC));
    // lift/gantry presets for autoalign
    new Trigger(
            () ->
                RobotOdometry.instance
                            .getPose("Main")
                            .getTranslation()
                            .getDistance(getTarget().getTranslation())
                        < 1.3
                    && dynamicAlign.isEnabled()
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(setupAutoPlace(() -> coralPreset));
    // coral place routine for autoalign
    // new Trigger(() -> coralAutoAlignWeight.isAutoalignComplete())
    //     .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kRightRumble, 1)));
    // followPathReef.generateTrigger(
    //     () ->
    //         driveController.a().getAsBoolean()
    //             && !followPathReef.isAutoalignComplete());
    DriveWeightCommand.createWeightTrigger(
        dynamicAlign,
        () -> driveController.a().getAsBoolean() && !dynamicAlign.globalAlignComplete());
    followPathCoral.generateTrigger(
        () -> driveHID.getLeftBumperButton() && !followPathCoral.isAutoalignComplete());

    new Trigger(
            () ->
                coralOuttakeSubsystem.hasCoral()
                    && coralOuttakeCommandFactory.ranBack
                    && !coralOuttakeSubsystem.guillotineCheck())
        .onTrue(setupAutoPlace(() -> CoralPreset.PreMove).onlyIf(() -> premoveLift));

    driveController.povDown().onTrue(new InstantCommand(() -> premoveLift = true));
    driveController.povUp().onTrue(new InstantCommand(() -> premoveLift = false));
    new Trigger(
            () ->
                followPathReef.isAutoalignComplete()
                    && liftSubsystem.isAtPreset(presetActive)
                    && gantryPresetActive != CoralPreset.Safe
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(getAutoPlaceCommand());

    new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.03)
        .whileTrue(
            climberCommandFactory.elevatorApplyVoltageCommand(
                () -> -operatorController.getRightY() * 4));

    new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.03)
        .whileTrue(
            climberCommandFactory.winchApplyVoltageCommand(
                () ->
                    -Math.signum(operatorController.getLeftY())
                        * Math.pow(operatorController.getLeftY(), 2)
                        * 12));

    // new Trigger(
    //         () ->
    //             followPathNearest.isAutoalignComplete()
    //                 && liftSubsystem.isAtPreset(CoralPreset.Safe.lift)
    //                 && algae.povRight().onTrue(climberCommandFactory.setClampState(() ->
    // false));Subsystem.hasAlgae()
    //                 && Robot.getState() != RobotState.AUTONOMOUS)
    //     .whileTrue(
    //         algaeCommandFactory
    //             .processCommand()
    //             .beforeStarting(() -> joystickDriveWeight.setEnabled(false))
    //             .finallyDo(() -> joystickDriveWeight.setEnabled(true)));
    new Trigger(() -> presetBoard.povIsUpwards())
        .onTrue(new InstantCommand(() -> algaeMode = true));

    new Trigger(() -> presetBoard.povIsDownwards())
        .onTrue(new InstantCommand(() -> algaeMode = false));

    new Trigger(() -> algaeIntakeSubsystem.hasAlgae() && Robot.getState() != RobotState.AUTONOMOUS)
        .whileTrue(algaeCommandFactory.setMotorVoltages(() -> 0.5, () -> 0.5));

    // new Trigger(
    //         () ->
    //             RobotOdometry.instance
    //                         .getPose("Main")
    //                         .getTranslation()
    //                         .getDistance(getTarget().getTranslation())
    //                     > 1.5
    //                 && !coralOuttakeCommandFactory.outtaking)
    //     .onTrue(runLiftToSafe());

    driveController.back().onTrue(new InstantCommand(() -> autoRampPos = !autoRampPos));

    // DriveWeightCommand.createWeightTrigger(
    //     new RotateToAngleWeight(
    //         () -> RobotOdometry.instance.getPose("Main"),
    //         () ->
    //             DistanceManager.getNearestPosition(
    //                     RobotOdometry.instance.getPose("Main"),
    //                     AllianceManager.chooseFromAlliance(
    //                         FieldConstants.coralStationPosBlue,
    // FieldConstants.coralStationPosRed))
    //                 .getRotation()
    //                 .plus(Rotation2d.fromDegrees(180))),
    //     driveController.leftBumper());

    // driveController
    //     .povDown()
    //     .onTrue(new InstantCommand(() -> joystickDriveWeight.setEnabled(true)));

    driveController
        .b()
        .whileTrue(
            coralOuttakeCommandFactory
                .outtake()
                .finallyDo(() -> coralOuttakeCommandFactory.outtaking = false));
    // operatorController.b().whileTrue(liftCommandFactory.runLiftMotionProfile(() -> 0.1));
    // rumble
    new Trigger(
            () ->
                coralOuttakeSubsystem.hasCoral()
                    && Robot.getState()
                        == RobotState.TELEOP /* driveController.getHID().getXButton() */)
        .onTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kLeftRumble, 0.4)))
        .onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kLeftRumble, 0)));
    new Trigger(
            () ->
                algaeIntakeSubsystem.hasAlgae()
                    && Robot.getState()
                        == RobotState.TELEOP /* driveController.getHID().getYButton() */)
        .onTrue(
            new InstantCommand(() -> operatorController.setRumble(RumbleType.kRightRumble, 0.2)))
        .onFalse(
            new InstantCommand(() -> operatorController.setRumble(RumbleType.kRightRumble, 0)));
    // new Trigger(() -> coralPreset.isRight())

    // reset gyro
    driveController.start().onTrue(gyro.resetGyroCommand());

    // gantry button bindings:
    operatorController.x().whileTrue(getAutoPlaceCommand());
    // operatorController
    //     .x()
    //     .onTrue(climberCommandFactory.setClampState(() -> !climberSubsystem.getSolenoidState()));
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
    // preset board
    new Trigger(() -> presetBoard.getLl2())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.LeftL2;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getRl2())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.RightL2;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getLl3())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.LeftL3;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getRl3())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.RightL3;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getLl4())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.LeftL4;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getRl4())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.RightL4;
                  algaeMode = false;
                }));
    new Trigger(() -> presetBoard.getTrough())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.LeftL3;
                  algaeMode = true;
                }));
    new Trigger(() -> presetBoard.getShare())
        .onTrue(
            new InstantCommand(
                () -> {
                  coralPreset = CoralPreset.LeftL2;
                  algaeMode = true;
                }));
    // lift/gantry manual controls
    operatorController
        .start()
        .whileTrue(
            new InstantCommand(() -> liftSubsystem.resetEncoder())
                .alongWith(new InstantCommand(() -> climberSubsystem.resetEncoder())));
    operatorController.a().onTrue(setupAutoPlace(() -> coralPreset));

    new Trigger(
            () ->
                (!coralOuttakeSubsystem.hasCoral() && Robot.getState() != RobotState.AUTONOMOUS)
                    && !coralOuttakeCommandFactory.outtaking)
        .onTrue(runLiftToSafe());

    // new Trigger(
    //         () ->
    //             algaeIntakeSubsystem.hasAlgae()
    //                 && RobotOdometry.instance
    //                         .getPose("Main")
    //                         .getTranslation()
    //                         .getDistance(getTarget().getTranslation())
    //                     > 0.3)
    //     .onTrue(runLiftToSafe());
    operatorController.b().onTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 0));

    new Trigger(() -> liftSubsystem.getIsLimited())
        .onTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 0));
    // TODO: THIS IS THE
    // ELEVATOR LIMIT!!!!!!

    // operatorController.b().whileTrue(climberCommandFactory.setWinchPosPID(() -> 90));
    // operatorController.b().whileTrue(climberCommandFactory.setElevatorPosPID(() -> -30));
    operatorController.povRight().onTrue(climberCommandFactory.setClampState(() -> false));
    operatorController.y().and(() -> !coralOuttakeCommandFactory.outtaking).onTrue(runLiftToSafe());

    driveController
        .y()
        .onTrue(
            new InstantCommand(() -> AntiTipWeight.setAntiTipEnabled(!AntiTipWeight.getEnabled())));

    driveController
        .rightTrigger()
        .and(() -> !algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(() -> true)
                .andThen(algaeCommandFactory.setMotorVoltages(() -> 4, () -> 4)))
        .onTrue(setupAutoPlace(() -> CoralPreset.Pickup));

    operatorController
        .rightTrigger()
        .and(() -> algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(() -> true)
                .andThen(algaeCommandFactory.processCommand()));
    // motor board
    new Trigger(() -> motorBoard.getLl2())
        .whileTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 2));
    new Trigger(() -> motorBoard.getLl3())
        .whileTrue(gantryCommandFactory.gantryApplyVoltageCommand(() -> 1));
    new Trigger(() -> motorBoard.getLl4())
        .whileTrue(coralOuttakeCommandFactory.setIntakeVoltage(() -> 4));
    new Trigger(() -> motorBoard.getRl4())
        .whileTrue(algaeCommandFactory.setMotorVoltages(() -> 10, () -> 10));
    new Trigger(() -> motorBoard.getRl3())
        .whileTrue(climberCommandFactory.elevatorApplyVoltageCommand(() -> -1));
    new Trigger(() -> motorBoard.getTrough())
        .onTrue(new InstantCommand(() -> liftSubsystem.resetEncoder()));

    new Trigger(operatorController.leftTrigger())
        .whileTrue(new InstantCommand(() -> algaeIntakeSubsystem.setHasAlgae(false)));

    // climber button bindings:
    operatorController
        .povUp()
        .whileTrue(
            climberRoutines.setupClimb().alongWith(new InstantCommand(() -> autoRampPos = false)));
    operatorController.povDown().whileTrue(climberCommandFactory.setWinchPosPIDFast(() -> 22));

    Command cancelCommand =
        (climberCommandFactory
            .setClampState(() -> false)
            .alongWith(new InstantCommand(() -> autoRampPos = false)));
    operatorController.povLeft().whileTrue(climberCommandFactory.setClampState(() -> true));
    // new Trigger(operatorController.leftTrigger())
    //     .whileTrue(liftCommandFactory.liftApplyVoltageCommand(() -> -1));

    // new Trigger(operatorController.rightTrigger())
    //     .whileTrue(liftCommandFactory.liftApplyVoltageCommand(() -> 1));

    // climber rumble
    new Trigger(() -> climberRoutines.isReadyToClamp())
        .onTrue(
            new InstantCommand(() -> operatorControllerHID.setRumble(RumbleType.kBothRumble, 1)))
        .onFalse(
            new InstantCommand(() -> operatorControllerHID.setRumble(RumbleType.kBothRumble, 0)));
    mapPIDtoCommand();
    if (TestConfig.testingMode == TestingSetting.pit) {
      configurePitBindings();
    } else {
      // put in TESTBOARD triggers here
    }
  }

  private void configurePitBindings() {
    new Trigger(() -> (pitController.getHID().getPOV() == 0))
        .whileTrue(
            climberCommandFactory.winchApplyVoltageCommand(
                (pitController.getHID().getPOV() == 0 ? () -> -2.5 : () -> 2.5)));
    new Trigger(() -> (pitController.getHID().getPOV() == 180))
        .whileTrue(
            climberCommandFactory.winchApplyVoltageCommand(
                (pitController.getHID().getPOV() == 180 ? () -> 2.5 : () -> -2.5)));

    new Trigger(() -> pitController.getHID().getPOV() == 270)
        .whileTrue(new InstantCommand(() -> autoRampPos = false));
    new Trigger(() -> Math.abs(pitController.getRightY()) > 0.03)
        .whileTrue(
            climberCommandFactory.elevatorApplyVoltageCommand(
                () -> -pitController.getRightY() * 4));
    pitController
        .rightBumper()
        .whileTrue(gantryCommandFactory.gantrySetVelocityCommand(() -> GantryConstants.alignSpeed));
    pitController
        .leftBumper()
        .whileTrue(
            gantryCommandFactory.gantrySetVelocityCommand(() -> -GantryConstants.alignSpeed));
    pitController
        .rightTrigger()
        .and(() -> !algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(() -> true)
                .andThen(algaeCommandFactory.setMotorVoltages(() -> 4, () -> 4)));
    pitController
        .leftTrigger()
        .and(() -> algaeIntakeSubsystem.hasAlgae())
        .whileTrue(
            algaeCommandFactory
                .setSolenoidState(() -> true)
                .andThen(algaeCommandFactory.processCommand()));
    pitController
        .start()
        .whileTrue(
            new InstantCommand(() -> liftSubsystem.resetEncoder())
                .alongWith(new InstantCommand(() -> climberSubsystem.resetEncoder()))
                .alongWith(new InstantCommand(() -> autoRampPos = true)));
    pitController
        .b()
        .whileTrue(
            coralOuttakeCommandFactory
                .outtake()
                .finallyDo(() -> coralOuttakeCommandFactory.outtaking = false));
    pitController.y().and(() -> !coralOuttakeCommandFactory.outtaking).onTrue(runLiftToSafe());
    pitController.back().whileTrue(gantryCommandFactory.gantryHomeCommand());
    pitController
        .povRight()
        .onTrue(climberCommandFactory.setClampState(() -> !climberSubsystem.getSolenoidState()));
    pitController.a().onTrue(setupAutoPlace(() -> coralPreset));
    pitController.x().onTrue(new InstantCommand(() -> AntiTipWeight.setAntiTipEnabled(false)));
  }

  public Command getAutonomousCommand() {
    return homing()
        .andThen(new InstantCommand(() -> autoRampPos = true))
        .andThen(new InstantCommand(() -> premoveLift = true))
        .andThen(dashboard.getAutoChooserCommand());
    // return new InstantCommand();
  }

  public Command homing() {
    return new ConditionalCommand(
        new InstantCommand(),
        (gantryCommandFactory
                .gantryHomeCommand()
                .onlyIf(() -> !gantrySubsystem.isLimitSwitchPressed()))
            .alongWith(new InstantCommand(() -> homed = true)),
        // .alongWith(liftCommandFactory.liftHomeCommand())
        // .alongWith(climberCommandFactory.liftHomeCommand()),
        () -> Robot.getMode() == Mode.SIM);
  }

  public Pose2d getTarget() {
    Pose2d[] array = chooseAlignPos();
    Pose2d x = DistanceManager.getNearestPosition(RobotOdometry.instance.getPose("Main"), array);
    return coralAdjust(
        DistanceManager.addRotatedDim(
            x, (algaeMode ? 0 : RobotDimensions.robotLength / 2), x.getRotation()),
        () -> coralPreset);
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

  public Command autonAutoPlace(Supplier<CoralPreset> coralPreset) {
    return liftCommandFactory
        .runLiftMotionProfile(
            () -> algaeMode ? coralPreset.get().getLiftAlgae() : coralPreset.get().getLift())
        .alongWith(
            autoScoringCommandFactory.gantryAlignCommand(
                coralPreset, () -> RobotOdometry.instance.getPose("MainTrig")))
        .alongWith(climberCommandFactory.setClampState(() -> false))
        .onlyIf(() -> !coralOuttakeSubsystem.guillotineCheck());
  }

  public Command setupAutoPlace(Supplier<CoralPreset> coralPreset) {
    return new InstantCommand(
            () -> {
              (new InstantCommand(
                          () -> {
                            presetActive =
                                algaeMode
                                    ? coralPreset.get().getLiftAlgae()
                                    : coralPreset.get().getLift();
                            gantryPresetActive = coralPreset.get();
                          })
                      .andThen(
                          liftCommandFactory.runLiftMotionProfile(() -> presetActive).asProxy())
                      .alongWith(
                          autoScoringCommandFactory
                              .gantryAlignCommand(
                                  () -> gantryPresetActive,
                                  () -> RobotOdometry.instance.getPose("MainTrig"))
                              .asProxy()))
                  .alongWith(climberCommandFactory.setClampState(() -> false))
                  .schedule();
            })
        .onlyIf(() -> !coralOuttakeSubsystem.guillotineCheck());
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
    return setupAutoPlace(() -> CoralPreset.Safe);
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

    NamedCommands.registerCommand(
        "HoldAlgae", algaeCommandFactory.setMotorVoltages(() -> 0.5, () -> 0.5));
    NamedCommands.registerCommand(
        "Process",
        algaeCommandFactory
            .processCommand()
            .andThen(algaeCommandFactory.setSolenoidState(() -> false)));
    NamedCommands.registerCommand("RunBackCoral", coralOuttakeCommandFactory.runBack());
    NamedCommands.registerCommand(
        "WaitForCoral",
        (new WaitUntilCommand(() -> (coralOuttakeSubsystem.guillotineCheck())))
            .deadlineFor(coralOuttakeCommandFactory.outtake()));

    NamedCommands.registerCommand("RunToPreset", autonAutoPlace(() -> coralPreset));
    NamedCommands.registerCommand("Safe", autonAutoPlace(() -> CoralPreset.Safe));
    NamedCommands.registerCommand(
        "WaitForPreset",
        new WaitUntilCommand(
                () ->
                    liftSubsystem.isAtPreset(
                        algaeMode ? coralPreset.getLiftAlgae() : coralPreset.getLift()))
            .deadlineFor(autonAutoPlace(() -> coralPreset))
            .deadlineFor(new PrintCommand("waiting...").repeatedly()));
    NamedCommands.registerCommand(
        "AutoReef",
        new WaitCommand(0)
            .andThen(getPlaceCommand())
            .deadlineFor(
                liftCommandFactory.runLiftMotionProfile(
                    () -> algaeMode ? coralPreset.getLiftAlgae() : coralPreset.getLift())));

    NamedCommands.registerCommand("PlaceTrough", autoScoringCommandFactory.placeTrough());
    NamedCommands.registerCommand(
        "logtest", new InstantCommand(() -> Logger.recordOutput("logtest", true)));

    NamedCommands.registerCommand(
        "LocalAlign",
        localAlign
            .getAutoCommand()
            .deadlineFor(autonAutoPlace(() -> coralPreset))
            .until(() -> localAlign.isAutoalignComplete() || !localAlign.isReady()));
    NamedCommands.registerCommand("WaitForLocal", new WaitUntilCommand(() -> localAlign.isReady()));
  }
}
