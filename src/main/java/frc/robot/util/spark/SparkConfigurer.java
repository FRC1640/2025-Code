package frc.robot.util.spark;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class SparkConfigurer {

  /*
   * May later necessitate can timeout parameter.
   */
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
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth())
            || (statusFrames.getFlashNecessary(spark)));
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
      PIDConstants PID) {
        SparkMax config = configSparkMax(id, idleMode, inverted, smartCurrentLimit, encoderMeasurementPeriod, encoderAverageDepth, statusFrames);
        config.configAccessor.;
        return config;
      }

  public static SparkMax configSparkMax(SparkConfiguration config) {
    return configSparkMax(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurmentPeriod(),
        config.getAverageEncoderDepth(),
        config.getStatusFrames(),
        config.getPID());
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    return configSparkFlex(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurmentPeriod(),
        config.getAverageEncoderDepth(),
        config.getStatusFrames());
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
        (inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth())
            || (statusFrames.getFlashNecessary(spark));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

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
        (inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth())
            || (statusFrames.getFlashNecessary(spark));
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
        (inverted != spark.configAccessor.getInverted())
            || (idleMode != spark.configAccessor.getIdleMode())
            || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
            || (encoderMeasurementPeriod
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth())
            || (statusFrames.getFlashNecessary(spark));
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
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
}
