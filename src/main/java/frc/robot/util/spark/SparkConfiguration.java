package frc.robot.util.spark;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

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

  public void applyMaxMotionConfig(MAXMotionConfig config) {
    inner.closedLoop.apply(config);
  }

  public SparkConfiguration applyPIDConfig(SparkPIDConstants sparkPIDConstant) {
    String loggerName = "Spark-" + id + "-" + sparkPIDConstant.closedLoopSlot.toString();
    if (sparkPIDConstant.alias != null) {
      loggerName = sparkPIDConstant.alias;
    }
    if (sparkPIDConstant.kP != null) {
      inner.closedLoop.p(sparkPIDConstant.kP);
      Logger.recordOutput("SparkPID/" + loggerName + "/Settings/kP", sparkPIDConstant.kP);
    }
    if (sparkPIDConstant.kI != null) {
      inner.closedLoop.i(sparkPIDConstant.kI);
      Logger.recordOutput("SparkPID/" + loggerName + "/Settings/kI", sparkPIDConstant.kI);
    }
    if (sparkPIDConstant.kD != null) {
      inner.closedLoop.d(sparkPIDConstant.kD);
      Logger.recordOutput("SparkPID/" + loggerName + "/Settings/kD", sparkPIDConstant.kD);
    }
    if (sparkPIDConstant.minOutput != null
        && sparkPIDConstant.maxOutput != null
        && sparkPIDConstant.closedLoopSlot != null) {
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/Settings/minOutput", sparkPIDConstant.minOutput);
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/Settings/maxOutput", sparkPIDConstant.maxOutput);

      inner.closedLoop.outputRange(
          sparkPIDConstant.minOutput, sparkPIDConstant.maxOutput, sparkPIDConstant.closedLoopSlot);
    } else if (sparkPIDConstant.minOutput != null && sparkPIDConstant.maxOutput != null) {
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/Settings/minOutput", sparkPIDConstant.minOutput);
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/Settings/maxOutput", sparkPIDConstant.maxOutput);
      inner.closedLoop.outputRange(sparkPIDConstant.minOutput, sparkPIDConstant.maxOutput);

    } else {
      inner.closedLoop.outputRange(-1, 1);
      Logger.recordOutput("SparkPID/" + loggerName + "/Settings/minOutput", -1);
      Logger.recordOutput("SparkPID/" + loggerName + "/Settings/maxOutput", 1);
    }
    if (sparkPIDConstant.velocityFF != null) {
      Logger.recordOutput("SparkPID/" + loggerName + "/velocityFF", sparkPIDConstant.velocityFF);
      inner.closedLoop.velocityFF(sparkPIDConstant.velocityFF);
    }
    if (sparkPIDConstant.maxVel != null) {
      Logger.recordOutput("SparkPID/" + loggerName + "/maxVel", sparkPIDConstant.maxVel);

      inner.closedLoop.maxMotion.maxVelocity(sparkPIDConstant.maxVel);
    }
    if (sparkPIDConstant.maxAccel != null) {
      Logger.recordOutput("SparkPID/" + loggerName + "/maxAccel", sparkPIDConstant.maxAccel);

      inner.closedLoop.maxMotion.maxAcceleration(sparkPIDConstant.maxAccel);
    }
    if (sparkPIDConstant.allowedErr != null) {
      Logger.recordOutput("SparkPID/" + loggerName + "/allowedError", sparkPIDConstant.allowedErr);

      inner.closedLoop.maxMotion.allowedClosedLoopError(sparkPIDConstant.allowedErr);
    }
    if (sparkPIDConstant.maxPositionMode != null) {
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/Settings/maxPos",
          sparkPIDConstant.maxPositionMode.toString());

      inner.closedLoop.maxMotion.positionMode(sparkPIDConstant.maxPositionMode);
    }
    if (sparkPIDConstant.positionConversionFactor != null) {
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/positionConversitionFactor",
          sparkPIDConstant.positionConversionFactor);

      inner.analogSensor.positionConversionFactor(sparkPIDConstant.positionConversionFactor);
    }
    if (sparkPIDConstant.velocityConversionFactor != null) {
      Logger.recordOutput(
          "SparkPID/" + loggerName + "/velocityConversionFactor",
          sparkPIDConstant.velocityConversionFactor);

      inner.analogSensor.velocityConversionFactor(sparkPIDConstant.velocityConversionFactor);
    }

    return this;
  }

  public void follow(SparkMax leader) {
    inner.follow(leader, leader.configAccessor.getInverted() != inverted);
  }

  public void follow(SparkMax leader, boolean inverted) {
    inner.follow(leader, inverted);
  }

  public void follow(SparkFlex leader) {
    inner.follow(leader, leader.configAccessor.getInverted() != inverted);
  }

  public void follow(SparkFlex leader, boolean inverted) {
    inner.follow(leader, inverted);
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
