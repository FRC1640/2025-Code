package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants.PivotId;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.logging.LogRunner;
import frc.robot.util.logging.VelocityLogStorage;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Module {
  ModuleIO io;
  PivotId id;
  private DoubleSupplier accelLimit;
  private DoubleSupplier deaccelLimit;
  ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  SlewRateLimiter accelLimiter = new SlewRateLimiter(accelLimit.getAsDouble());
  SlewRateLimiter deaccelLimiter = new SlewRateLimiter(deaccelLimit.getAsDouble());

  public Module(ModuleIO io, PivotId id, DoubleSupplier accelLimit, DoubleSupplier deaccelLimit) {
    this.accelLimit = accelLimit;
    this.deaccelLimit = deaccelLimit;
    this.io = io;
    this.id = id;
    AlertsManager.addAlert(
        () -> !inputs.driveConnected, id.toString() + " drive disconnected.", AlertType.kError);
    AlertsManager.addAlert(
        () -> !inputs.steerConnected, id.toString() + " steer disconnected.", AlertType.kError);
    AlertsManager.addAlert(
        () -> inputs.driveCurrentAmps > WarningThresholdConstants.maxVortexMotorCurrent,
        id.toString() + " drive motor over-current.",
        AlertType.kWarning);
    AlertsManager.addAlert(
        () -> inputs.steerCurrentAmps > WarningThresholdConstants.maxVortexMotorCurrent,
        id.toString() + " steer motor over-current.",
        AlertType.kWarning);
    AlertsManager.addAlert(
        () -> inputs.driveTempCelsius > WarningThresholdConstants.maxMotorTemp,
        id.toString() + " drive motor is hot.",
        AlertType.kWarning);
    AlertsManager.addAlert(
        () -> inputs.steerTempCelsius > WarningThresholdConstants.maxMotorTemp,
        id.toString() + " steer motor is hot.",
        AlertType.kWarning);
    LogRunner.addLog(
        new VelocityLogStorage(() -> getVelocity(), () -> io.velocitySetpoint(), "driveVelocity"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Modules/" + id, inputs);
  }

  public void setDesiredStateMetersPerSecond(SwerveModuleState state) {

    if (Math.abs(state.speedMetersPerSecond) <= 0.005) {
      io.setDriveVelocity(0, inputs);
      io.setSteerVoltage(0);
      return;
    }
    boolean flipDriveTeleop = false;
    Rotation2d delta = state.angle.minus(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
    if (Math.abs(delta.getDegrees()) > 90.0) {
      flipDriveTeleop = true;
    }

    Rotation2d angle =
        (flipDriveTeleop) ? state.angle.plus(Rotation2d.fromDegrees(180)) : state.angle;

    io.setSteerPosition(angle, inputs);

    double targetSpeed =
        (flipDriveTeleop ? state.speedMetersPerSecond : -state.speedMetersPerSecond)
            * Math.abs(Math.cos(delta.getRadians()));

    if (Math.signum(targetSpeed - inputs.driveVelocityMetersPerSecond) != Math.signum(targetSpeed)
        || targetSpeed == 0) {
      targetSpeed = deaccelLimiter.calculate(targetSpeed);
      accelLimiter.reset(targetSpeed);
    } else {
      targetSpeed = accelLimiter.calculate(targetSpeed);
      deaccelLimiter.reset(targetSpeed);
    }

    io.setDriveVelocity(targetSpeed, inputs);
  }

  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  public SwerveModulePosition[] getOdometryPositions() {
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    SwerveModulePosition[] odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsMeters[i];
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
    return odometryPositions;
  }

  public SwerveModuleState[] getModuleStates() {
    int sampleCount = inputs.odometryTimestamps.length;
    SwerveModuleState[] odometrySpeeds = new SwerveModuleState[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double velocity = inputs.driveVelocities[i];
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometrySpeeds[i] = new SwerveModuleState(velocity, angle);
    }
    return odometrySpeeds;
  }

  public void stop() {
    io.setDriveVoltage(0);
    io.setSteerVoltage(0);
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public void setSteerVoltage(double volts) {
    io.setSteerVoltage(volts);
  }

  public double getDriveVoltage() {
    return inputs.driveAppliedVoltage;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityMetersPerSecond,
        new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionMeters, new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
  }

  public double getVelocity() {
    return inputs.driveVelocityMetersPerSecond;
  }
}
