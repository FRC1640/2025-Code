package frc.robot.util.spark;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkConfiguration {
  private int id;
  private IdleMode idleMode;
  private boolean inverted;
  private int currentLimit;
  private int encoderMeasurmentPeriod;
  private int averageEncoderDepth;
  private StatusFrames statusFrames;
  private Optional<Double> P;
  private Optional<Double> I;
  private Optional<Double> D;

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

  public Optional<Double> getP() {
    return P;
  } 

  public Optional<Double> getI() {
    return I;
  } 

  public Optional<Double> getD() {
    return P;
  } 

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurmentPeriod,
      int averageEncoderDepth,
      StatusFrames statusFrames,
      Optional<Double> P,
      Optional<Double> I,
      Optional<Double> D) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurmentPeriod = encoderMeasurmentPeriod;
    this.averageEncoderDepth = averageEncoderDepth;
    this.statusFrames = statusFrames;
    this.P = P;
    this.I = I;
    this.D = D;
  }
}
