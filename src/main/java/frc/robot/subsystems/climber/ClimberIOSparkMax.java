package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

  DoubleSolenoid doubleSolenoid;

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
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
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
    winch1Motor.setVoltage(clampVoltage(applyLimits(inputs.winchMotor1Position, voltage)));
    winch2Motor.setVoltage(clampVoltage(applyLimits(inputs.winchMotor2Position, voltage)));
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
    inputs.winchMotor1Position = winch1Encoder.getPosition();
    inputs.winchMotor2Position = winch2Encoder.getPosition();
    inputs.liftMotorVelocity = liftEncoder.getVelocity();
    inputs.winchMotor1Velocity = winch1Encoder.getVelocity();
    inputs.winchMotor2Velocity = winch2Encoder.getVelocity();
    inputs.liftMotorCurrent = liftMotor.getOutputCurrent();
    inputs.winchMotor1Current = winch1Motor.getOutputCurrent();
    inputs.winchMotor2Current = winch2Motor.getOutputCurrent();
    inputs.liftMotorVoltage = liftMotor.getAppliedOutput();
    inputs.winchMotor1Voltage = winch1Motor.getAppliedOutput();
    inputs.winchMotor2Voltage = winch2Motor.getAppliedOutput();
    inputs.liftMotorTemperature = liftMotor.getMotorTemperature();
    inputs.winchMotor1Temperature = winch1Motor.getMotorTemperature();
    inputs.winchMotor2Temperature = winch2Motor.getMotorTemperature();
  }

  @Override
  public void setSolenoidState(boolean forward, ClimberIOInputs inputs) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
