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
  private Optional<PIDConstants> pid;
  private Optional<LimitSwitchConfig> limitSwitch;
  private SparkBaseConfig inner;

  public void limitSwitch(LimitSwitchConfig limitSwitchConfig) {
    limitSwitch = Optional.of(limitSwitchConfig);
  }

  public boolean configuresLimitSwitch() {
    return limitSwitch.isPresent();
  }

  public void pid(PIDConstants pid) {
    this.pid = Optional.of(pid);
  }

  public boolean configuresPid() {
    return pid.isPresent();
  }

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
    return pid;
  }
  
  public PIDConstants retrievePID() {
    return pid.get();
  }

  public Optional<LimitSwitchConfig> getLimitSwitch() {
    return limitSwitch;
  }

  public LimitSwitchConfig retrieveLimitSwitch() {
    return limitSwitch.get();
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
      Optional<PIDConstants> pid,
      Optional<LimitSwitchConfig> limitSwitch,
      SparkMaxConfig seed) {
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
    seed.absoluteEncoder.averageDepth(encoderAverageDepth);
    seed.alternateEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    seed.encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    if (pid.isPresent()) {
      PIDConstants pidConstants = pid.get();
      inner.closedLoop.p(pidConstants.kP).i(pidConstants.kI).d(pidConstants.kD);
    }
    if (limitSwitch.isPresent()) {
      ;
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
    this.id = id;
    this.idleMode = idleMode;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.encoderMeasurementPeriod = encoderMeasurementPeriod;
    this.encoderAverageDepth = encoderAverageDepth;
    this.statusFrames = statusFrames;
    this.pid = Optional.empty();
    this.limitSwitch = Optional.empty();
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
}
