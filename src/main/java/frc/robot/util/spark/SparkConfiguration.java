package frc.robot.util.spark;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfiguration {
  private int id;
  private IdleMode idleMode;
  private boolean inverted;
  private int currentLimit;
  private int encoderMeasurementPeriod;
  private int encoderAverageDepth;
  private StatusFrames statusFrames;
  private PIDConstants pid;
  private SparkBaseConfig inner;

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

  public int getEncoderAverageDepth() {
    return encoderAverageDepth;
  }

  public StatusFrames getStatusFrames() {
    return statusFrames;
  }

  public SparkBaseConfig getInnerConfig() {
    return inner;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      SparkMaxConfig seed) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = new PIDConstants(1); // TODO is this a reasonable default?
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    seed.alternateEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    inner = seed;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      SparkFlexConfig seed) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = new PIDConstants(1); // TODO is this a reasonable default?
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    seed.externalEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    inner = seed;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      PIDConstants pid,
      SparkMaxConfig seed) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = pid;
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    seed.alternateEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    seed.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD);
    inner = seed;
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      PIDConstants pid,
      SparkFlexConfig seed) {
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = pid;
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    seed.externalEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    seed.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD);
    inner = seed;
  }
}
