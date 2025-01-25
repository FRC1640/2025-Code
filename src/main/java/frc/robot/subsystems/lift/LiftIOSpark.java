package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  RelativeEncoder motorEncoder1;
  RelativeEncoder motorEncoder2;

  public LiftIOSpark() {
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor1ID, false));
    SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftMotor2ID, false));
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {}
}
