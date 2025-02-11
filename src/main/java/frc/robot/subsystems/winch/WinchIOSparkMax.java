package frc.robot.subsystems.winch;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.util.tools.MotorLim;

public class WinchIOSparkMax implements WinchIO{
  
  private final SparkMax winchLeaderSpark;
  private final SparkMax winchFollowerSpark;
  /*
   * Set voltage of the winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {
    winchLeaderSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchLeaderMotorPosition, voltage, ClimberConstants.winchLimits)));
  }
  /*
   * Sets the position of the winch motors using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, ClimberIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.liftMotorPosition, position)), inputs);
  }
}
