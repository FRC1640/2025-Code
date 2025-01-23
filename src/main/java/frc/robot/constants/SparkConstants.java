package frc.robot.constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.StatusFrames;

public class SparkConstants {
  public static final SparkConfiguration getDefaultMax(int id) {
    return new SparkConfiguration(
        id, IdleMode.kCoast, false, 60, 8, 2, StatusFrames.getDefault(), new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultFlex(int id) {
    return new SparkConfiguration(
        id, IdleMode.kCoast, false, 60, 8, 2, StatusFrames.getDefault(), new SparkFlexConfig());
  }

  public static final SparkFlex driveFlex(int id) {
    return SparkConfigurer.configSparkFlex(
        new SparkConfiguration(
            id,
            getDefaultFlex(id).getIdleMode(),
            true,
            80,
            8,
            2,
            new StatusFrames(
                100, 20, (int) (1000 / DriveConstants.odometryFrequency), 500, 500, 500, 500),
            new SparkFlexConfig()));
  }
}
