package frc.robot.util.spark;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkConfiguration {
  private int id;
  private IdleMode idleMode;
  private boolean inverted;
  private int currentLimit;
  private int encoderMeasurmentPeriod;
  private int averageEncoderDepth;
  private StatusFrames statusFrames;

  public int getId() {
    return id;
  }

  public IdleMode getIdleMode() {
    return idleMode;
  }

  public boolean isInverted() {
    return inverted;
  }

  public int getCurrentLimit() {
    return currentLimit;
  }

  public int getEncoderMeasurmentPeriod() {
    return encoderMeasurmentPeriod;
  }

  public int getAverageEncoderDepth() {
    return averageEncoderDepth;
  }

  public StatusFrames getStatusFrames() {
    return statusFrames;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurmentPeriod,
      int averageEncoderDepth,
      StatusFrames statusFrames) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurmentPeriod = encoderMeasurmentPeriod;
    this.averageEncoderDepth = averageEncoderDepth;
    this.statusFrames = statusFrames;
  }
}
