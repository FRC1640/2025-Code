package frc.robot.subsystems.lift;

import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  public LiftIOSpark() {
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor1, false));
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor2, false));
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {}
}
