package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.PivotId;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.sysid.SwerveDriveSysidRoutine;
import java.util.Arrays;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriveSubsystem extends SubsystemBase {
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  Gyro gyro;
  SysIdRoutine sysIdRoutine;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

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
                new SysIdRoutine.Config(Volts.per(Seconds).of(2), Volts.of(8), Seconds.of(15)));

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }

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
    for (var module : modules) {
      module.periodic();
    }
    gyro.periodic();
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
  private ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getActualSwerveStates());
  }

  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds percent =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond / DriveConstants.maxSpeed,
            speeds.vyMetersPerSecond / DriveConstants.maxSpeed,
            speeds.omegaRadiansPerSecond / DriveConstants.maxOmega);

    ChassisSpeeds doubleCone = doubleCone(percent, new Translation2d());
    ChassisSpeeds speedsOptimized =
        new ChassisSpeeds(
            doubleCone.vxMetersPerSecond * DriveConstants.maxSpeed,
            doubleCone.vyMetersPerSecond * DriveConstants.maxSpeed,
            doubleCone.omegaRadiansPerSecond * DriveConstants.maxOmega);

    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speedsOptimized, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );

    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredStateMetersPerSecond(previousSetpoint.moduleStates()[i]);
    }
  }

  public static ChassisSpeeds doubleCone(
      ChassisSpeeds speedsPercent, Translation2d centerOfRotation) {

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
      k = Math.max(linearRotSpeed, translationalSpeed) / (linearRotSpeed + translationalSpeed);
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
