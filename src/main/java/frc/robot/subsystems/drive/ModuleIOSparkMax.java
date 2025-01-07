package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.sensors.odometry.SparkOdometryThread;
import frc.robot.sensors.resolvers.ResolverVoltage;
import java.util.Queue;

public class ModuleIOSparkMax implements ModuleIO {
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> driveVelocityQueue;

  private final RelativeEncoder driveEncoder;
  private final ResolverVoltage steeringEncoder;

  private final SparkFlex driveSpark;
  private final SparkMax steerSpark;

  private final PIDController drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.drivePID);
  private final SimpleMotorFeedforward driveFF =
      RobotPIDConstants.constructFF(RobotPIDConstants.driveFF);
  private final PIDController steerPID = RobotPIDConstants.constructPID(RobotPIDConstants.steerPID);

  public ModuleIOSparkMax(ModuleInfo id) {
    driveSpark = SparkConstants.driveFlex(id.driveChannel);
    steerSpark = SparkConstants.getDefaultSparkMax(id.steerChannel);
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(driveSpark, () -> driveSpark.getEncoder().getPosition());

    driveEncoder = driveSpark.getEncoder();
    steeringEncoder =
        new ResolverVoltage(
            id.resolverChannel,
            DriveConstants.initalSlope,
            DriveConstants.finalSlope,
            180.0,
            90.0,
            id.angleOffset);
    driveVelocityQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(driveSpark, () -> driveEncoder.getVelocity());

    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(steerSpark, () -> steeringEncoder.getDegrees());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        -(driveEncoder.getPosition() / DriveConstants.driveGearRatio)
            * DriveConstants.wheelRadius
            * 2
            * Math.PI;
    inputs.driveVelocityMetersPerSecond =
        -((driveEncoder.getVelocity() / DriveConstants.driveGearRatio) / 60)
            * 2
            * Math.PI
            * DriveConstants.wheelRadius;
    inputs.driveAppliedVoltage =
        driveSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
    inputs.driveTempCelsius = driveSpark.getMotorTemperature();
    inputs.steerAngleDegrees = steeringEncoder.getDegrees();
    inputs.steerAppliedVoltage =
        steerSpark.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.steerCurrentAmps = steerSpark.getOutputCurrent();
    inputs.steerTempCelsius = steerSpark.getMotorTemperature();
    inputs.steerEncoderVoltage = steeringEncoder.getVoltage();
    inputs.steerAngleDegrees =
        (360 - (steerSpark.getEncoder().getPosition() / DriveConstants.steerGearRatio * 360)) % 360;

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    -(value / DriveConstants.driveGearRatio)
                        * DriveConstants.wheelRadius
                        * 2
                        * Math.PI)
            .toArray();

    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    inputs.driveVelocities =
        driveVelocityQueue.stream()
            .mapToDouble(
                (Double value) ->
                    -(value / DriveConstants.driveGearRatio)
                        / 60
                        * DriveConstants.wheelRadius
                        * 2
                        * Math.PI)
            .toArray();

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    driveVelocityQueue.clear();

    inputs.rawEncoderValue = steeringEncoder.getRawValue();
  }

  @Override
  public void setDriveVelocity(double velocity, ModuleIOInputs inputs) {
    double pidSpeed = driveFF.calculate(velocity);
    pidSpeed += drivePID.calculate(inputs.driveVelocityMetersPerSecond, velocity);
    setDriveVoltage(pidSpeed);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveSpark.setVoltage(voltage);
  }

  @Override
  public void setSteerPosition(Rotation2d angle, ModuleIOInputs inputs) {
    Rotation2d delta = angle.minus(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
    double sin = Math.sin(delta.getRadians());
    setSteerVoltage(steerPID.calculate(sin, 0) * 12);
  }

  @Override
  public void setSteerVoltage(double voltage) {
    steerSpark.setVoltage(voltage);
  }
}
