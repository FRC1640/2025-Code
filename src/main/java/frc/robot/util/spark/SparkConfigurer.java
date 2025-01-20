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
    boolean[] flashComponents = new boolean[5];
    flashComponents[0] = (inverted != spark.configAccessor.getInverted());
    flashComponents[1] = (idleMode != spark.configAccessor.getIdleMode());
    flashComponents[2] = (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit());
    flashComponents[3] =
        (encoderMeasurementPeriod != spark.configAccessor.encoder.getQuadratureMeasurementPeriod());
    flashComponents[4] =
        (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth());
    /* boolean flash =
    (inverted != spark.configAccessor.getInverted())
        || (idleMode != spark.configAccessor.getIdleMode())
        || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
        || (encoderMeasurementPeriod
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()); */
    boolean flash =
        flashComponents[0]
            || flashComponents[1]
            || flashComponents[2]
            || flashComponents[3]
            || flashComponents[4];
    for (int i = 0; i < 5; i++) {
      Logger.recordOutput(
          "SparkFlashes/IndividualChecks/Id" + id + "/Check" + i, flashComponents[i]);
    }
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  public static SparkMax configSparkMax(SparkConfiguration config) {
    return configSparkMax(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurementPeriod(),
        config.getEncoderAverageDepth(),
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
    boolean[] flashComponents = new boolean[5];
    flashComponents[0] = (inverted != spark.configAccessor.getInverted());
    flashComponents[1] = (idleMode != spark.configAccessor.getIdleMode());
    flashComponents[2] = (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit());
    flashComponents[3] =
        (encoderMeasurementPeriod != spark.configAccessor.encoder.getQuadratureMeasurementPeriod());
    flashComponents[4] =
        (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth());
    /* boolean flash =
    (inverted != spark.configAccessor.getInverted())
        || (idleMode != spark.configAccessor.getIdleMode())
        || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
        || (encoderMeasurementPeriod
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()); */
    boolean flash =
        flashComponents[0]
            || flashComponents[1]
            || flashComponents[2]
            || flashComponents[3]
            || flashComponents[4];
    for (int i = 0; i < 5; i++) {
      Logger.recordOutput(
          "SparkFlashes/IndividualChecks/Id" + id + "/Check" + i, flashComponents[i]);
    }
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
    boolean[] flashComponents = new boolean[5];
    flashComponents[0] = (inverted != spark.configAccessor.getInverted());
    flashComponents[1] = (idleMode != spark.configAccessor.getIdleMode());
    flashComponents[2] = (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit());
    flashComponents[3] =
        (encoderMeasurementPeriod != spark.configAccessor.encoder.getQuadratureMeasurementPeriod());
    flashComponents[4] =
        (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth());
    /* boolean flash =
    (inverted != spark.configAccessor.getInverted())
        || (idleMode != spark.configAccessor.getIdleMode())
        || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
        || (encoderMeasurementPeriod
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()); */
    boolean flash =
        flashComponents[0]
            || flashComponents[1]
            || flashComponents[2]
            || flashComponents[3]
            || flashComponents[4];
    for (int i = 0; i < 5; i++) {
      Logger.recordOutput(
          "SparkFlashes/IndividualChecks/Id" + id + "/Check" + i, flashComponents[i]);
    }
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + id, flash);
    return spark;
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    return configSparkFlex(
        config.getId(),
        config.getIdleMode(),
        config.isInverted(),
        config.getCurrentLimit(),
        config.getEncoderMeasurementPeriod(),
        config.getEncoderAverageDepth(),
        config.getStatusFrames());
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
    boolean[] flashComponents = new boolean[5];
    flashComponents[0] = (inverted != spark.configAccessor.getInverted());
    flashComponents[1] = (idleMode != spark.configAccessor.getIdleMode());
    flashComponents[2] = (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit());
    flashComponents[3] =
        (encoderMeasurementPeriod != spark.configAccessor.encoder.getQuadratureMeasurementPeriod());
    flashComponents[4] =
        (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth());
    /* boolean flash =
    (inverted != spark.configAccessor.getInverted())
        || (idleMode != spark.configAccessor.getIdleMode())
        || (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit())
        || (encoderMeasurementPeriod
            != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
        || (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()); */
    boolean flash =
        flashComponents[0]
            || flashComponents[1]
            || flashComponents[2]
            || flashComponents[3]
            || flashComponents[4];
    for (int i = 0; i < 5; i++) {
      Logger.recordOutput(
          "SparkFlashes/IndividualChecks/Id" + id + "/Check" + i, flashComponents[i]);
    }
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
