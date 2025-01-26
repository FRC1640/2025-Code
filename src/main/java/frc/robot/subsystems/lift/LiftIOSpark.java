package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  RelativeEncoder leaderEncoder;
  RelativeEncoder followerEncoder;
  SparkMax leaderMotor;
  SparkMax followerMotor;

  public LiftIOSpark() {
    leaderMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftleaderMotorID, false));
    followerMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftfollowerMotorID, false), leaderMotor);
    leaderEncoder = leaderMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();
  }

  /*
   * Set speed % between -1 and 1
   */
  @Override
  public void setSpeed(double speed) {
    leaderMotor.set(clampSpeed(leaderEncoder.getPosition(), speed));
  }
  /*
   * Set voltage of the motor
   */
  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(clampSpeed(leaderEncoder.getPosition(), voltage));
  }
}
