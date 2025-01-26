package frc.robot.constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.StatusFrames;

public class SparkConstants {
  public static final SparkMax getDefaultSparkMax(int id) {
    return SparkConfigurer.configSparkMax(
        id, IdleMode.kCoast, false, 60, 200, 64, StatusFrames.getDefault());
  }

  public static final SparkMax getGantryDefaultSparkMax(int id) {
    return SparkConfigurer.configSparkMax(
        id, IdleMode.kBrake, false, 60, 200, 64, StatusFrames.getDefault());
  }

  public static final SparkFlex getDefaultSparkFlex(int id) {
    return SparkConfigurer.configSparkFlex(
        id, IdleMode.kCoast, false, 60, 200, 64, StatusFrames.getDefault());
  }

  public static final SparkConfiguration getDefault() {
    return new SparkConfiguration(
        -1, IdleMode.kCoast, false, 60, 200, 64, StatusFrames.getDefault());
  }

  public static final SparkFlex driveFlex(int id) {
    return SparkConfigurer.configSparkFlex(
        id,
        getDefault().getIdleMode(),
        true,
        80,
        8,
        2,
        new StatusFrames(
            100, 20, (int) (1000 / DriveConstants.odometryFrequency), 500, 500, 500, 500));
  }
}
