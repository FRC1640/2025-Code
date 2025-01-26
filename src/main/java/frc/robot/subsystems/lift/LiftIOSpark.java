package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  RelativeEncoder motorEncoder1;
  RelativeEncoder motorEncoder2;
  SparkMax motor1;
  SparkMax motor2;

  public LiftIOSpark() {
    motor1 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftMotor1ID, false));
    motor2 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftMotor2ID, false));
    motorEncoder1 = motor1.getEncoder();
    motorEncoder2 = motor2.getEncoder();
  }

  /*
   * Set speed % between -1 and 1
   */
  @Override
  public void setSpeed(double speed) {
    motor1.set(clampSpeed(motorEncoder1.getPosition(), speed));
  }
  /*
   * Set voltage of the motor
   */
  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(clampSpeed(motorEncoder1.getPosition(), voltage));
  }
}
