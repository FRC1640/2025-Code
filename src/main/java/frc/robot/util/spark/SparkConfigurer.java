package frc.robot.util.spark;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class SparkConfigurer {
  public static SparkMax configSparkMax(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames) {
    SparkMax spark = new SparkMax(id, MotorType.kBrushless);
    SparkMaxConfig config =
        buildSparkMaxConfig(
            idleMode,
            inverted,
            smartCurrentLimit,
            encoderMeasurementPeriod,
            encoderAverageDepth,
            statusFrames);
    boolean flash =
        ((inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  public static SparkMax configSparkMax(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      LimitSwitchConfig limitSwitch) {
    SparkMax spark = new SparkMax(id, MotorType.kBrushless);
    SparkMaxConfig config =
        buildSparkMaxConfig(
            idleMode,
            inverted,
            smartCurrentLimit,
            encoderMeasurementPeriod,
            encoderAverageDepth,
            statusFrames);
    config.apply(limitSwitch);
    boolean flash =
        ((inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  public static SparkFlex configSparkFlex(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames) {
    SparkFlex spark = new SparkFlex(id, MotorType.kBrushless);
    SparkFlexConfig config =
        buildSparkFlexConfig(
            idleMode,
            inverted,
            smartCurrentLimit,
            encoderMeasurementPeriod,
            encoderAverageDepth,
            statusFrames);
    boolean flash =
        ((inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  public static SparkFlex configSparkFlex(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      LimitSwitchConfig limitSwitch) {
    SparkFlex spark = new SparkFlex(id, MotorType.kBrushless);
    SparkFlexConfig config =
        buildSparkFlexConfig(
            idleMode,
            inverted,
            smartCurrentLimit,
            encoderMeasurementPeriod,
            encoderAverageDepth,
            statusFrames);
    config.apply(limitSwitch);
    boolean flash =
        ((inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  // TODO via configuration
  public static SparkMax configSparkMax(SparkConfiguration config) {
    return configSparkMax(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurementPeriod(),
        config.getAverageEncoderDepth(),
        config.getStatusFrames(),
        config.getPid());
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    return configSparkFlex(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurementPeriod(),
        config.getAverageEncoderDepth(),
        config.getStatusFrames());
  }

  // TODO build
  private static SparkMaxConfig buildSparkMaxConfig(
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode).inverted(inverted).smartCurrentLimit(smartCurrentLimit);
    config.absoluteEncoder.averageDepth(encoderAverageDepth);
    config
        .alternateEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    config
        .encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    statusFrames.apply(config.signals);
    return config;
  }

  private static SparkFlexConfig buildSparkFlexConfig(
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(idleMode).inverted(inverted).smartCurrentLimit(smartCurrentLimit);
    config.absoluteEncoder.averageDepth(encoderAverageDepth);
    config
        .externalEncoder
        .averageDepth(encoderAverageDepth)
        .measurementPeriod(encoderMeasurementPeriod);
    config
        .encoder
        .quadratureAverageDepth(encoderAverageDepth)
        .quadratureMeasurementPeriod(encoderMeasurementPeriod);
    statusFrames.apply(config.signals);
    return config;
  }

  // TODO get if flash
  private static boolean getFlash(SparkConfiguration config, SparkMax spark) {
    return ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod()
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (config.getAverageEncoderDepth()
            != spark.configAccessor.encoder.getQuadratureAverageDepth()));
  }

  private static boolean getFlashPid(SparkConfiguration config, SparkMax spark) {
    return ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod()
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (config.getAverageEncoderDepth()
            != spark.configAccessor.encoder.getQuadratureAverageDepth())
        || (config.getPid().kP != spark.configAccessor.closedLoop.getP())
        || (config.getPid().kI != spark.configAccessor.closedLoop.getI())
        || (config.getPid().kD != spark.configAccessor.closedLoop.getD()));
  }

  private static boolean getFlash(SparkConfiguration config, SparkFlex spark) {
    return ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod()
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (config.getAverageEncoderDepth()
            != spark.configAccessor.encoder.getQuadratureAverageDepth()));
  }

  private static boolean getFlashPid(SparkConfiguration config, SparkFlex spark) {
    return ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod()
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (config.getAverageEncoderDepth()
            != spark.configAccessor.encoder.getQuadratureAverageDepth())
        || (config.getPid().kP != spark.configAccessor.closedLoop.getP())
        || (config.getPid().kI != spark.configAccessor.closedLoop.getI())
        || (config.getPid().kD != spark.configAccessor.closedLoop.getD()));
  }
}
