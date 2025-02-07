package frc.robot.sensors.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.odometry.SparkOdometryThread;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
  double offset = 0;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> rate;

  double pitchOffset = 0;
  double rollOffset = 0;

  public GyroIONavX() {
    rate = SparkOdometryThread.getInstance().registerSignal(() -> Math.toRadians(gyro.getRate()));
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(() -> gyro.getRotation2d().getRadians());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.isConnected = gyro.isConnected();
    inputs.isCalibrating = gyro.isCalibrating();
    inputs.angleRadiansRaw = gyro.getRotation2d().getRadians();
    inputs.angularVelocityDegreesPerSecond = gyro.getRate();
    inputs.angleDegreesRaw = Math.toDegrees(inputs.angleRadiansRaw);

    inputs.roll = new Rotation2d(gyro.getRotation3d().getX() - rollOffset);
    inputs.pitch = new Rotation2d(gyro.getRotation3d().getY() - pitchOffset);

    inputs.displacementX = gyro.getDisplacementX();
    inputs.displacementY = gyro.getDisplacementY();
    inputs.odometryYawRate = rate.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRadians(value))
            .toArray(Rotation2d[]::new);

    inputs.accelX = gyro.getWorldLinearAccelX();
    inputs.accelY = gyro.getWorldLinearAccelY();
    inputs.accelZ = gyro.getWorldLinearAccelZ();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    rate.clear();
  }

  @Override
  public void resetGyro(GyroIOInputs inputs) {
    rollOffset = inputs.roll.getRadians();
    pitchOffset = inputs.pitch.getRadians();
    offset = inputs.angleRadiansRaw;
  }

  @Override
  public double getActual(GyroIOInputs inputs) {
    return inputs.angleRadiansRaw - offset;
  }

  @Override
  public double getOffset() {
    return offset;
  }

  @Override
  public void setOffset(double offset) {
    this.offset = offset;
  }
}
