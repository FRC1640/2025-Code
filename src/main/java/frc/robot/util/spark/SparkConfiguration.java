package frc.robot.util.spark;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkConfiguration {
  private int id;
  private IdleMode idleMode;
  private boolean inverted;
  private int currentLimit;
  private int encoderMeasurementPeriod;
  private int averageEncoderDepth;
  private StatusFrames statusFrames;
  private PIDConstants pid;

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

  public int getEncoderMeasurementPeriod() {
    return encoderMeasurementPeriod;
  }

  public int getAverageEncoderDepth() {
    return averageEncoderDepth;
  }

  public StatusFrames getStatusFrames() {
    return statusFrames;
  }

  public PIDConstants getPid() {
    return pid;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int averageEncoderDepth,
      StatusFrames statusFrames,
      PIDConstants pid) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.averageEncoderDepth = averageEncoderDepth;
    this.statusFrames = statusFrames;
    this.pid = pid;
  }
}
