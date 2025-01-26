package frc.robot.subsystems.lift;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  RelativeEncoder leaderEncoder;
  RelativeEncoder followerEncoder;
  SparkMax leaderMotor;
  SparkMax followerMotor;
  PIDController liftPID;

  public LiftIOSpark() {
    liftPID = PIDConstants.constructPID(RobotPIDConstants.liftPID);
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
   * Set voltage of the motor
   */
  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(clampSpeed(leaderEncoder.getPosition(), voltage));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setPosition(double position) {}
}
