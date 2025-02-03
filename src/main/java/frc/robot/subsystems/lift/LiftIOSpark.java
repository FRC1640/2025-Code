package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  ElevatorFeedforward elevatorFeedforward =
      RobotPIDConstants.constructFFElevator(RobotPIDConstants.liftFF);

  ProfiledPIDController profiledPIDController =
      RobotPIDConstants.costructProfiledPIDController(
          RobotPIDConstants.liftProfiledPIDConstants, LiftConstants.constraints);

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
            MotorLim.applyLimits(
                inputs.leaderMotorPosition,
                voltage,
                LiftConstants.liftLimits.low,
                LiftConstants.liftLimits.high)));
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
  public void setLiftPositionMotionProfile(double position, LiftIOInputs inputs) {
    profiledPIDController.setGoal(position);
    setLiftVoltage(
        MotorLim.clampVoltage(
            profiledPIDController.calculate(inputs.leaderMotorPosition)
                + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity)),
        inputs);
  }

  @Override
  public void resetLiftMotionProfile(LiftIOInputs inputs) {
    profiledPIDController.reset(inputs.leaderMotorPosition);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.leaderMotorPosition =
        leaderEncoder.getPosition()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2;
    inputs.followerMotorPosition =
        followerEncoder.getPosition()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2;
    inputs.leaderMotorVelocity =
        leaderEncoder.getVelocity()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2
            / 60;
    inputs.followerMotorVelocity =
        followerEncoder.getVelocity()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2
            / 60;
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
