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
  private final SparkMax winchLeaderSpark;
  private final SparkMax winchFollowerSpark;
  private final ResolverPWM absoluteEncoder;
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID, "winchPID");
  private final PIDController winchAnglePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchAnglePID, "winchAnglePID");

  public WinchIOSparkMax() {
    winchLeaderSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false, true));
    winchFollowerSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false),
            winchLeaderSpark);

    winchPID.enableContinuousInput(0, 360);
    absoluteEncoder =
        new ResolverPWM(
            ClimberConstants.absoluteEncoderChannel, ClimberConstants.absoluteEncoderOffset);
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

  /*
   * Sets the angle of the winch motors using a PID
   * angle counts up clockwise starting at 0 degrees = due west
   */
  @Override
  public void setClimberWinchAngle(double angle, WinchIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchAnglePID.calculate(inputs.winchAngle, angle)), inputs);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    inputs.winchAngle = (absoluteEncoder.getDegrees()) % 360;
    inputs.winchLeaderMotorPosition = winchLeaderSpark.getEncoder().getPosition();
    inputs.winchLeaderMotorCurrent = winchLeaderSpark.getOutputCurrent();
    inputs.winchFollowerMotorCurrent = winchFollowerSpark.getOutputCurrent();
    inputs.winchLeaderMotorVoltage = winchLeaderSpark.getAppliedOutput();
    inputs.winchFollowerMotorVoltage = winchFollowerSpark.getAppliedOutput();
    inputs.winchLeaderMotorTemperature = winchLeaderSpark.getMotorTemperature();
    inputs.winchFollowerMotorTemperature = winchFollowerSpark.getMotorTemperature();
  }
}
