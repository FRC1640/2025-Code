package frc.robot.subsystems.winch;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.sensors.resolvers.ResolverPWM;
import frc.robot.util.misc.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class WinchIOSparkMax implements WinchIO {
  private final SparkMax winchSpark1;
  private final SparkMax winchSpark2;
  private final ResolverPWM absoluteEncoder;
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID, "winchPID");
  private final PIDController winchAnglePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchAnglePID, "winchAnglePID");

  public WinchIOSparkMax() {
    winchSpark1 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false));
    winchSpark2 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false));

    winchPID.enableContinuousInput(0, 360);
    absoluteEncoder =
        new ResolverPWM(
            ClimberConstants.absoluteEncoderChannel, ClimberConstants.absoluteEncoderOffset);
  }

  /*
   * Set voltage of winch motor 1
   */
  @Override
  public void setClimberWinch1Voltage(double voltage, WinchIOInputs inputs) {
    winchSpark1.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winch1MotorPosition,
                voltage,
                ClimberConstants.winchLimits.low,
                ClimberConstants.winchLimits.high)));
  }
  /*
   * Set voltage of winch motor 2
   */
  @Override
  public void setClimberWinch2Voltage(double voltage, WinchIOInputs inputs) {
    winchSpark2.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winch1MotorPosition,
                voltage,
                ClimberConstants.winchLimits.low,
                ClimberConstants.winchLimits.high)));
  }
  /*
   * Set voltage of both winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {
    setClimberWinch1Voltage(voltage, inputs);
    setClimberWinch2Voltage(voltage, inputs);
  }
  /*
   * Sets the position of the winch motors using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, WinchIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.winch1MotorPosition, position)), inputs);
  }

  /*
   * Sets the angle of the winch motors using a PID
   * PIDs the relative positions of the motors to 0 while running
   * angle counts up clockwise starting at 0 degrees = due west
   */
  @Override
  public void setClimberWinchAngle(double angle, WinchIOInputs inputs) {
    setClimberWinch1Voltage(
        MotorLim.clampVoltage(
            winchAnglePID.calculate(inputs.winchAngle, angle)
                + winchPID.calculate(inputs.winch1MotorPosition, inputs.winch2MotorPosition)
                    * ClimberConstants.positionSyncPIDMultiplier),
        inputs);
    setClimberWinch2Voltage(
        MotorLim.clampVoltage(
            winchAnglePID.calculate(inputs.winchAngle, angle)
                + winchPID.calculate(inputs.winch2MotorPosition, inputs.winch1MotorPosition)
                    * ClimberConstants.positionSyncPIDMultiplier),
        inputs);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    inputs.winchAngle = (absoluteEncoder.getDegrees()) % 360;
    inputs.winch1MotorPosition = winchSpark1.getEncoder().getPosition();
    inputs.winch1MotorCurrent = winchSpark1.getOutputCurrent();
    inputs.winch2MotorCurrent = winchSpark2.getOutputCurrent();
    inputs.winch1MotorVoltage = winchSpark1.getAppliedOutput();
    inputs.winch2MotorVoltage = winchSpark2.getAppliedOutput();
    inputs.winch1MotorTemperature = winchSpark1.getMotorTemperature();
    inputs.winch2MotorTemperature = winchSpark2.getMotorTemperature();
  }
}
