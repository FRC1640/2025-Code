package frc.robot.subsystems.winch;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.misc.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class WinchIOSparkMax implements WinchIO {
  private final SparkMax winchLeaderSpark;
  private final SparkMax winchFollowerSpark;
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID, "winchPID");

  public WinchIOSparkMax() {
    winchLeaderSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false, true));
    winchFollowerSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false));

    winchPID.enableContinuousInput(0, 360);
  }
  /*
   * Set voltage of the winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {
    winchFollowerSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchFollowerMotorPosition,
                voltage,
                ClimberConstants.winchLimits.low,
                ClimberConstants.winchLimits.high)));
  }
  /*
   * Sets the position of the winch motors using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, WinchIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.winchLeaderMotorPosition, position)),
        inputs);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    inputs.winchLeaderMotorPosition =
        360 - (winchLeaderSpark.getAbsoluteEncoder().getPosition() * 360) % 360;
    inputs.winchLeaderMotorCurrent = winchLeaderSpark.getOutputCurrent();
    inputs.winchFollowerMotorCurrent = winchFollowerSpark.getOutputCurrent();
    inputs.winchLeaderMotorVoltage = winchLeaderSpark.getAppliedOutput();
    inputs.winchFollowerMotorVoltage = winchFollowerSpark.getAppliedOutput();
    inputs.winchLeaderMotorTemperature = winchLeaderSpark.getMotorTemperature();
    inputs.winchFollowerMotorTemperature = winchFollowerSpark.getMotorTemperature();
  }
}
