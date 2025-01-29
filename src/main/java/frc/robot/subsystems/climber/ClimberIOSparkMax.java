package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class ClimberIOSparkMax implements ClimberIO {
  RelativeEncoder liftEncoder;
  RelativeEncoder winch1Encoder;
  RelativeEncoder winch2Encoder;
  SparkMax liftMotor;
  SparkMax winch1Motor;
  SparkMax winch2Motor;
  PIDController liftController = RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);
  PIDController winchController = RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);

  public ClimberIOSparkMax() {
    liftMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberLiftMotorID, false));
    winch1Motor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false));
    winch2Motor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false));
    liftEncoder = liftMotor.getEncoder();
    winch1Encoder = winch1Motor.getEncoder();
    winch2Encoder = winch2Motor.getEncoder();
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {
    liftMotor.setVoltage(clampVoltage(applyLimits(inputs.liftMotorPosition, voltage)));
  }
  /*
   * Sets the position of the lift motor using a PID
   */
  @Override
  public void setClimberLiftPosition(double position, ClimberIOInputs inputs) {
    setClimberLiftVoltage(
        clampVoltage(liftController.calculate(inputs.liftMotorPosition, position)), inputs);
  }
  /*
   * Set voltage of the winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {
    winch1Motor.setVoltage(clampVoltage(applyLimits(inputs.winch1MotorPosition, voltage)));
    winch2Motor.setVoltage(clampVoltage(applyLimits(inputs.winch2MotorPosition, voltage)));
  }
  /*
   * Sets the position of the winch motors using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, ClimberIOInputs inputs) {
    setClimberWinchVoltage(
        clampVoltage(winchController.calculate(inputs.liftMotorPosition, position)), inputs);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.liftMotorPosition = liftEncoder.getPosition();
    inputs.winch1MotorPosition = winch1Encoder.getPosition();
    inputs.winch2MotorPosition = winch2Encoder.getPosition();
    inputs.liftMotorVelocity = liftEncoder.getVelocity();
    inputs.winch1MotorVelocity = winch1Encoder.getVelocity();
    inputs.winch2MotorVelocity = winch2Encoder.getVelocity();
    inputs.liftMotorCurrent = liftMotor.getOutputCurrent();
    inputs.winch1MotorCurrent = winch1Motor.getOutputCurrent();
    inputs.winch2MotorCurrent = winch2Motor.getOutputCurrent();
    inputs.liftMotorVoltage = liftMotor.getAppliedOutput();
    inputs.winch1MotorVoltage = winch1Motor.getAppliedOutput();
    inputs.winch2MotorVoltage = winch2Motor.getAppliedOutput();
    inputs.liftTemperature = liftMotor.getMotorTemperature();
    inputs.winch1Temperature = winch1Motor.getMotorTemperature();
    inputs.winch2Temperature = winch2Motor.getMotorTemperature();
  }
}
