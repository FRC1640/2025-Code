package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.odometry.RobotOdometry;

public class GyroIOSim implements GyroIO {

  Rotation2d angle = new Rotation2d();

  double offset;

  public GyroIOSim() {}

  @Override
  public void resetGyro(GyroIOInputs inputs) {
    offset = inputs.angleRadiansRaw;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.angleRadiansRaw = RobotOdometry.instance.getPose("Main").getRotation().getRadians();
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
