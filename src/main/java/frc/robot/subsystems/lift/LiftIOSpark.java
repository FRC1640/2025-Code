package frc.robot.subsystems.lift;

import frc.robot.constants.SparkConstants;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  public LiftIOSpark() {
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor1));
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor2));
  }
  
  @Override 
  public void updateInputs(LiftIOInputs inputs) {}
}
