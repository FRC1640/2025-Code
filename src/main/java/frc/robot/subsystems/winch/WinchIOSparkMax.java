package frc.robot.subsystems.winch;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.sensors.resolvers.ResolverPWM;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class WinchIOSparkMax implements WinchIO {

  private final ResolverPWM winchEncoder;
  private final SparkMax winchLeaderSpark;
  private final SparkMax winchFollowerSpark;
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);

  public WinchIOSparkMax() {
    winchLeaderSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false));
    winchFollowerSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false),
            winchLeaderSpark);

    winchEncoder = new ResolverPWM(-1, 0);
  }
  /*
   * Set voltage of the winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {
    winchLeaderSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchLeaderMotorPosition,
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
        winchEncoder.getDegrees(); // says degrees but really in meters
    inputs.winchLeaderMotorCurrent = winchLeaderSpark.getOutputCurrent();
    inputs.winchFollowerMotorCurrent = winchFollowerSpark.getOutputCurrent();
    inputs.winchLeaderMotorVoltage = winchLeaderSpark.getAppliedOutput();
    inputs.winchFollowerMotorVoltage = winchFollowerSpark.getAppliedOutput();
    inputs.winchLeaderMotorTemperature = winchLeaderSpark.getMotorTemperature();
    inputs.winchFollowerMotorTemperature = winchFollowerSpark.getMotorTemperature();
  }
}
