package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

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
  public void setLiftVoltage(double voltage, LiftIOInputs inputs) {
    leaderMotor.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(inputs.leaderMotorPosition, voltage, LiftConstants.liftLimits)));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setLiftPosition(double position, LiftIOInputs inputs) {
    setLiftVoltage(
        MotorLim.clampVoltage(liftController.calculate(inputs.leaderMotorPosition, position)),
        inputs);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.leaderMotorPosition = leaderEncoder.getPosition();
    inputs.followerMotorPosition = followerEncoder.getPosition();
    inputs.leaderMotorVelocity = leaderEncoder.getVelocity();
    inputs.followerMotorVelocity = followerEncoder.getVelocity();
    inputs.leaderMotorCurrent = leaderMotor.getOutputCurrent();
    inputs.followerMotorCurrent = followerMotor.getOutputCurrent();
    inputs.leaderMotorVoltage =
        leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.followerMotorVoltage =
        followerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.leaderTemperature = leaderMotor.getMotorTemperature();
    inputs.followerTemperature = followerMotor.getMotorTemperature();
  }
}
