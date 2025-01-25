package frc.robot.util.spark;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.Optional;

public class SparkConfiguration {
  private int id;
  private IdleMode idleMode;
  private boolean inverted;
  private int currentLimit;
  private int encoderMeasurementPeriod;
  private int encoderAverageDepth;
  private StatusFrames statusFrames;
  private PIDConstants pid;
  private LimitSwitchConfig limitSwitch;
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

  public Optional<PIDConstants> getPID() {
    return Optional.ofNullable(pid);
  }

  public Optional<LimitSwitchConfig> getLimitSwitch() {
    return Optional.ofNullable(limitSwitch);
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
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        null,
        null,
        seed);
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
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        pid,
        null,
        seed);
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      LimitSwitchConfig limitSwitchConfig,
      SparkMaxConfig seed) {
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        null,
        limitSwitchConfig,
        seed);
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
      LimitSwitchConfig limitSwitch,
      SparkMaxConfig seed) {
    encoderMeasurementPeriod /= 2; // seems like this is doubled somehow
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = pid;
    this.limitSwitch = limitSwitch;
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    // seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    // seed.alternateEncoder
    //     .averageDepth(encoderAverageDepth)
    //     .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    if (pid != null) {
      seed.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD);
    }
    if (limitSwitch != null) {
      seed.limitSwitch.apply(limitSwitch);
    }
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
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        null,
        null,
        seed);
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
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        pid,
        null,
        seed);
  }

  public SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      LimitSwitchConfig limitSwitchConfig,
      SparkFlexConfig seed) {
    this(
        id,
        idleMode,
        inverted,
        currentLimit,
        encoderMeasurementPeriod,
        encoderAverageDepth,
        statusFrames,
        null,
        limitSwitchConfig,
        seed);
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
      LimitSwitchConfig limitSwitch,
      SparkFlexConfig seed) {
    encoderMeasurementPeriod /= 2; // seems like this is doubled somehow
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = pid;
    this.limitSwitch = limitSwitch;
    seed.idleMode(idleMode).inverted(inverted).smartCurrentLimit(currentLimit);
    // seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    // seed.externalEncoder
    //     .averageDepth(encoderAverageDepth)
    //     .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    if (pid != null) {
      seed.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD);
    }
    if (limitSwitch != null) {
      seed.limitSwitch.apply(limitSwitch);
    }
    inner = seed;
  }
}
