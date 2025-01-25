package frc.robot.subsystems.lift;

import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotConstants;

public class LiftIOSpark implements LiftIO {
  private SparkMax liftMotor1;
  private SparkMax liftMotor2;

  public LiftIOSpark() {
    liftMotor1 =
        new SparkMax(RobotConstants.LiftConstants.liftMotor1, SparkMax.MotorType.kBrushless);
    liftMotor2 =
        new SparkMax(RobotConstants.LiftConstants.liftMotor2, SparkMax.MotorType.kBrushless);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {}
}
