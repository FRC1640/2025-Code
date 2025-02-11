package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean isConnected = false;
    public boolean isCalibrating = false;
    public double angleRadiansRaw = 0.0;
    public double angularVelocityDegreesPerSecond = 0.0;
    public double angleDegreesRaw = 0.0;
    public double displacementX = 0.0;
    public double displacementY = 0.0;

    public Rotation2d roll;
    public Rotation2d pitch;

    public double accelX;
    public double accelY;
    public double accelZ;

    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] odometryYawRate = new double[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetGyro(GyroIOInputs inputs) {}

  public default double getActual(GyroIOInputs inputs) {
    return 0;
  }

  public default double getOffset() {
    return 0;
  }

  public default void setOffset(double offset) {}
}
