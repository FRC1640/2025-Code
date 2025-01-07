package frc.robot.util.spark;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class SparkMaxConfigurer {
  /* public enum EncoderType {
      ABSOLUTE,
      ALTERNATE,
      QUADRATURE;
  } */

  /*
   * May later necessitate can timeout parameter.
   */
  public static SparkMax configSpark(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int smartCurrentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames) {
    SparkMax spark = new SparkMax(id, MotorType.kBrushless);
    SparkMaxConfig config =
        buildConfig(
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

  public static SparkMax configSpark(
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
        buildConfig(
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

  private static SparkMaxConfig buildConfig(
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
    /* switch (encoderType) {
        case ABSOLUTE:
            config.absoluteEncoder.averageDepth(encoderAverageDepth);
            break;
        case ALTERNATE:
            config.alternateEncoder.averageDepth(encoderAverageDepth).measurementPeriod(encoderMeasurementPeriod);
            break;
        default:
            config.encoder.quadratureAverageDepth(encoderAverageDepth).quadratureMeasurementPeriod(encoderMeasurementPeriod);
            break;
    } */
    statusFrames.apply(config.signals);
    return config;
  }
}

