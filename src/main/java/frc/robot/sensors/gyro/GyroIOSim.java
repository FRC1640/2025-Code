package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

public class GyroIOSim implements GyroIO {

  Rotation2d angle = new Rotation2d();

  double offset;

  private DoubleSupplier rotRate;

  public GyroIOSim(DoubleSupplier rotRate) {
    this.rotRate = rotRate;
  }

  @Override
  public void resetGyro(GyroIOInputs inputs) {
    offset = inputs.angleRadiansRaw;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.isCalibrating = false;
    inputs.angleRadiansRaw = inputs.angleRadiansRaw + rotRate.getAsDouble() * 0.02;
    angle = new Rotation2d(inputs.angleRadiansRaw);

    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {Rotation2d.fromRadians(inputs.angleRadiansRaw)};

    inputs.pitch = new Rotation2d(0);
    inputs.roll = new Rotation2d(0);
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
