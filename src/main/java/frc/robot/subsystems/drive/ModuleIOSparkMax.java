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
import frc.robot.sensors.resolvers.ResolverPWM;
import frc.robot.util.spark.SparkConfigurer;
import java.util.Queue;

public class ModuleIOSparkMax implements ModuleIO {
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> driveVelocityQueue;

  private final RelativeEncoder driveEncoder;
  private final ResolverPWM steeringEncoder;
  // private final ResolverVoltage steeringEncoder;

  private final SparkFlex driveSpark;
  private final SparkMax steerSpark;

  private final PIDController drivePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.drivePID, "DrivePID");
  private final SimpleMotorFeedforward driveFF =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.driveFF);
  private final PIDController steerPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.steerPID, "SteerPID");

  public ModuleIOSparkMax(ModuleInfo id) {
    driveSpark = SparkConstants.driveFlex(id.driveChannel);
    steerSpark =
        SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(id.steerChannel, true));
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(driveSpark, () -> driveSpark.getEncoder().getPosition());

    driveEncoder = driveSpark.getEncoder();
    steeringEncoder = new ResolverPWM(id.resolverChannel, id.angleOffset);
    // steeringEncoder =
    //     new ResolverVoltage(
    //         id.resolverChannel,
    //         DriveConstants.initalSlope,
    //         DriveConstants.finalSlope,
    //         180.0,
    //         90.0,
    //         id.angleOffset);
    driveVelocityQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(driveSpark, () -> driveEncoder.getVelocity());

    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(steerSpark, () -> steeringEncoder.getDegrees() % 360);
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
    inputs.steerAppliedVoltage =
        steerSpark.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.steerCurrentAmps = steerSpark.getOutputCurrent();
    inputs.steerRadPerSec =
        steerSpark.getEncoder().getVelocity() * Math.PI * 2 / 60 / DriveConstants.steerGearRatio;
    inputs.steerTempCelsius = steerSpark.getMotorTemperature();
    // inputs.steerEncoderRawValue = steeringEncoder.getFrequency();
    inputs.steerEncoderRelative =
        (360 - (steerSpark.getEncoder().getPosition() / DriveConstants.steerGearRatio * 360)) % 360;
    inputs.steerAngleDegrees = (steeringEncoder.getDegrees()) % 360;

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

    // inputs.rawEncoderValue = steeringEncoder.getRawValue();
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
