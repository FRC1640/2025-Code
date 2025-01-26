package frc.robot.util.spark;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

public class SparkConfigurer {
  public static SparkMax configSparkMax(SparkConfiguration config) {
    SparkMax spark = new SparkMax(config.getId(), MotorType.kBrushless);
    boolean flash = getFlash(config, spark);
    spark.configure(
        config.getInnerConfig(),
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + config.getId(), flash);
    return spark;
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    SparkFlex spark = new SparkFlex(config.getId(), MotorType.kBrushless);
    boolean flash = getFlash(config, spark);
    spark.configure(
        config.getInnerConfig(),
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + config.getId(), flash);
    return spark;
  }

  private static boolean getFlash(SparkConfiguration config, SparkMax spark) {
    boolean flash =
        ((config.isInverted() != spark.configAccessor.getInverted())
            || (config.getIdleMode() != spark.configAccessor.getIdleMode())
            || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
            || (config.getEncoderMeasurementPeriod()
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (config.getEncoderAverageDepth()
                != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    if (config.getPID().isPresent()) {
      flash =
          (flash
              || (config.getPID().get().kP != spark.configAccessor.closedLoop.getP())
              || (config.getPID().get().kI != spark.configAccessor.closedLoop.getI())
              || (config.getPID().get().kD != spark.configAccessor.closedLoop.getD()));
    }
    return flash;
  }

  private static boolean getFlash(SparkConfiguration config, SparkFlex spark) {
    boolean flash =
        ((config.isInverted() != spark.configAccessor.getInverted())
            || (config.getIdleMode() != spark.configAccessor.getIdleMode())
            || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
            || (config.getEncoderMeasurementPeriod()
                != spark.configAccessor.encoder.getQuadratureMeasurementPeriod())
            || (config.getEncoderAverageDepth()
                != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    if (config.getPID().isPresent()) {
      flash =
          (flash
              || (config.getPID().get().kP != spark.configAccessor.closedLoop.getP())
              || (config.getPID().get().kI != spark.configAccessor.closedLoop.getI())
              || (config.getPID().get().kD != spark.configAccessor.closedLoop.getD()));
    }
    return flash;
  }
}
