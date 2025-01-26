package frc.robot.subsystems.lift;

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
  PIDController liftController = RobotPIDConstants.constructPID(RobotPIDConstants.liftPID);

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
  public void setPosition(double position) {
    leaderMotor.setVoltage(liftController.calculate(leaderEncoder.getPosition(), position));
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.leadermotorPosition = leaderEncoder.getPosition();
    inputs.followermotorPosition = followerEncoder.getPosition();
    inputs.leadermotorVelocity = leaderEncoder.getVelocity();
    inputs.followermotorVelocity = followerEncoder.getVelocity();
    inputs.leadermotorCurrent = leaderMotor.getOutputCurrent();
    inputs.followermotorCurrent = followerMotor.getOutputCurrent();
    inputs.leadermotorVoltage = leaderMotor.getAppliedOutput();
    inputs.followermotorVoltage = followerMotor.getAppliedOutput();
  }
}
