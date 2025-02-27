package frc.robot.util.spark;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.logging.MotorTrack;
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
    createAlerts(spark.getFaults(), spark.getWarnings());
    MotorTrack.addSpark(config.getId(), spark);
    return spark;
  }

  /*
   * Configure a SparkMax with given SparkConfiguration config and what Leader it is set to
   */
  public static SparkMax configSparkMax(SparkConfiguration config, SparkMax leader) {
    config.follow(leader);
    SparkMax spark = configSparkMax(config);
    MotorTrack.addSpark(config.getId(), spark);
    return spark;
  }

  public static SparkMax configSparkMax(SparkConfiguration config, SparkFlex leader) {
    config.follow(leader);
    SparkFlex spark = configSparkFlex(config);
    MotorTrack.addSpark(config.getId(), spark);
    return configSparkMax(config);
  }
  /*
   * Configure a spark flex with the given configuration
   */
  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    SparkFlex spark = new SparkFlex(config.getId(), MotorType.kBrushless);
    boolean flash = getFlash(config, spark);
    spark.configure(
        config.getInnerConfig(),
        ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + config.getId(), flash);
    createAlerts(spark.getFaults(), spark.getWarnings());
    MotorTrack.addSpark(config.getId(), spark);
    return spark;
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkMax leader) {
    config.follow(leader);
    SparkFlex temp = configSparkFlex(config);
    MotorTrack.addSpark(config.getId(), temp);
    return temp;
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkFlex leader) {
    config.follow(leader);
    SparkFlex sparkFlex = configSparkFlex(config);
    MotorTrack.addSpark(config.getId(), sparkFlex);
    return sparkFlex;
  }
  /*
   * Check if the spark needs to be flashed for settings that are currently flashed
   */

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
  /*
   * Check if the spark needs to be flashed for settings that are currently flashed
   */

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

  private static void createAlerts(Faults faults, Warnings warnings) {
    AlertsManager.addAlert(() -> faults.can, "Spark CAN fatal error.", AlertType.kError);
    AlertsManager.addAlert(() -> faults.other, "Unknown spark fatal error.", AlertType.kError);
    AlertsManager.addAlert(
        () -> faults.motorType, "Spark motor type fatal error.", AlertType.kError);
    AlertsManager.addAlert(() -> faults.firmware, "Spark firmware fatal error.", AlertType.kError);
    AlertsManager.addAlert(
        () -> faults.gateDriver, "Spark gate driver fatal error.", AlertType.kError);
    AlertsManager.addAlert(() -> faults.sensor, "Spark sensor fatal error.", AlertType.kError);
    AlertsManager.addAlert(
        () -> faults.temperature, "Spark temperature fatal error.", AlertType.kError);
    AlertsManager.addAlert(
        () -> faults.escEeprom, "Spark speed controller ROM fatal error.", AlertType.kError);

    AlertsManager.addAlert(() -> warnings.brownout, "Spark brownout warning.", AlertType.kWarning);
    AlertsManager.addAlert(
        () -> warnings.overcurrent, "Spark over-current warning.", AlertType.kWarning);
    AlertsManager.addAlert(() -> warnings.sensor, "Spark sensor warning.", AlertType.kWarning);
    AlertsManager.addAlert(
        () -> warnings.escEeprom, "Spark speed controller ROM warning.", AlertType.kWarning);
    AlertsManager.addAlert(() -> warnings.stall, "Spark stall warning", AlertType.kWarning);
    AlertsManager.addAlert(() -> warnings.hasReset, "Spark reset warning", AlertType.kWarning);
    AlertsManager.addAlert(
        () -> warnings.extEeprom, "Spark external ROM warning", AlertType.kWarning);
  }
}
