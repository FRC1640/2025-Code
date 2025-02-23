package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.PivotId;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.weights.PathplannerWeight;
import frc.robot.util.pathplanning.LocalADStarAK;
import frc.robot.util.sysid.SwerveDriveSysidRoutine;
import frc.robot.util.tools.ChassisSpeedHelper;
import frc.robot.util.tools.RequirementHandler;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  Gyro gyro;
  SysIdRoutine sysIdRoutine;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  public static final Lock odometryLock = new ReentrantLock();

  public DriveSubsystem(Gyro gyro) {
    this.gyro = gyro;
    switch (Robot.getMode()) { // create modules
      case REAL:
        modules[0] = new Module(new ModuleIOSparkMax(DriveConstants.FL), PivotId.FL);
        modules[1] = new Module(new ModuleIOSparkMax(DriveConstants.FR), PivotId.FR);
        modules[2] = new Module(new ModuleIOSparkMax(DriveConstants.BL), PivotId.BL);
        modules[3] = new Module(new ModuleIOSparkMax(DriveConstants.BR), PivotId.BR);
        break;

      case SIM:
        modules[0] = new Module(new ModuleIOSim(DriveConstants.FL), PivotId.FL);
        modules[1] = new Module(new ModuleIOSim(DriveConstants.FR), PivotId.FR);
        modules[2] = new Module(new ModuleIOSim(DriveConstants.BL), PivotId.BL);
        modules[3] = new Module(new ModuleIOSim(DriveConstants.BR), PivotId.BR);
        break;

      default:
        modules[0] = new Module(new ModuleIO() {}, PivotId.FL);
        modules[1] = new Module(new ModuleIO() {}, PivotId.FR);
        modules[2] = new Module(new ModuleIO() {}, PivotId.BL);
        modules[3] = new Module(new ModuleIO() {}, PivotId.BR);
        break;
    }
    sysIdRoutine =
        new SwerveDriveSysidRoutine()
            .createNewRoutine(
                modules[0],
                modules[1],
                modules[2],
                modules[3],
                this,
                new SysIdRoutine.Config(
                    Volts.per(Seconds).of(2),
                    Volts.of(8),
                    Seconds.of(15),
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())));

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }
    AutoBuilder.configure(
        () -> RobotOdometry.instance.getPose("Main"),
        (x) -> {
          RobotOdometry.instance.setPose("Main", x);
          RobotOdometry.instance.resetGyro(x);
        },
        this::getChassisSpeeds,
        (x) -> PathplannerWeight.setSpeeds(x),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        new RequirementHandler());
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Path/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          PathplannerWeight.setpoint = targetPose;
          Logger.recordOutput("Drive/Path/TrajectorySetpoint", targetPose);
        });
    setpointGenerator =
        new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating
            // trajectories and running path following commands.
            DriveConstants.maxSteerSpeed);
    previousSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getActualSwerveStates(), DriveFeedforwards.zeros(4));
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    for (var module : modules) {
      module.periodic();
    }
    gyro.periodic();
    odometryLock.unlock();
  }

  public void stop() {
    for (Module m : modules) {
      m.stop();
    }
  }

  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  public SwerveModuleState[] getActualSwerveStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getActualSwerveStates());
  }

  public double chassisSpeedsMagnitude() {
    return ChassisSpeedHelper.magnitude(getChassisSpeeds());
  }

  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/VelocityAngle")
  public Rotation2d chassisSpeedsAngle() {
    return ChassisSpeedHelper.angleOf(getChassisSpeeds()).rotateBy(gyro.getAngleRotation2d());
  }

  public Module[] getModules() {
    return modules;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public void runVelocity(ChassisSpeeds speeds, boolean fieldCentric, double dreamLevel) {
    ChassisSpeeds percent =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond / DriveConstants.maxSpeed,
            speeds.vyMetersPerSecond / DriveConstants.maxSpeed,
            speeds.omegaRadiansPerSecond / DriveConstants.maxOmega);

    ChassisSpeeds doubleCone = inceptionMode(percent, new Translation2d(), dreamLevel);
    ChassisSpeeds speedsOptimized =
        fieldCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    doubleCone.vxMetersPerSecond * DriveConstants.maxSpeed,
                    doubleCone.vyMetersPerSecond * DriveConstants.maxSpeed,
                    doubleCone.omegaRadiansPerSecond * DriveConstants.maxOmega),
                gyro.getAngleRotation2d())
            : new ChassisSpeeds(
                doubleCone.vxMetersPerSecond * DriveConstants.maxSpeed,
                doubleCone.vyMetersPerSecond * DriveConstants.maxSpeed,
                doubleCone.omegaRadiansPerSecond * DriveConstants.maxOmega);

    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speedsOptimized, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );
    Logger.recordOutput("Drive/SwerveStates/SetpointStates", previousSetpoint.moduleStates());
    Logger.recordOutput(
        "Drive/SwerveStates/Input", DriveConstants.kinematics.toSwerveModuleStates(speeds));
    Logger.recordOutput(
        "Drive/SwerveStates/DoubleCone",
        DriveConstants.kinematics.toSwerveModuleStates(speedsOptimized));
    // speedsOptimized = ChassisSpeeds.discretize(speedsOptimized, 0.02);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredStateMetersPerSecond(previousSetpoint.moduleStates()[i]);
      // DriveConstants.kinematics.toSwerveModuleStates(speedsOptimized)[i]);
    }
  }

  public static ChassisSpeeds inceptionMode(
      ChassisSpeeds speedsPercent, Translation2d centerOfRotation, double dreamLevel) {

    double xSpeed = speedsPercent.vxMetersPerSecond;
    double ySpeed = speedsPercent.vyMetersPerSecond;
    double rot = speedsPercent.omegaRadiansPerSecond;
    double translationalSpeed = Math.hypot(xSpeed, ySpeed);
    double linearRotSpeed =
        Math.abs(rot * computeMaxNorm(DriveConstants.positions, centerOfRotation));
    double k;
    if (linearRotSpeed == 0 || translationalSpeed == 0) {
      k = 1;
    } else {
      k =
          Math.pow(
              Math.max(linearRotSpeed, translationalSpeed) / (linearRotSpeed + translationalSpeed),
              dreamLevel);
    }
    return new ChassisSpeeds(k * xSpeed, k * ySpeed, k * rot);
  }

  public static double computeMaxNorm(
      Translation2d[] translations, Translation2d centerOfRotation) {
    return Arrays.stream(translations)
        .map((translation) -> translation.minus(centerOfRotation))
        .mapToDouble(Translation2d::getNorm)
        .max()
        .orElseThrow(() -> new NoSuchElementException("No max norm."));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
