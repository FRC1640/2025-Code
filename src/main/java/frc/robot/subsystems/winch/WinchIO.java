package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO extends AutoCloseable {

  @AutoLog
  public static class WinchIOInputs {
    public double winchLeaderMotorPosition = 0.0;
    public double winchFollowerMotorPosition = 0.0;
    public double winchLeaderMotorVelocity = 0.0;
    public double winchFollowerMotorVelocity = 0.0;
    public double winchLeaderMotorCurrent = 0.0;
    public double winchFollowerMotorCurrent = 0.0;
    public double winchLeaderMotorVoltage = 0.0;
    public double winchFollowerMotorVoltage = 0.0;
    public double winchLeaderMotorTemperature = 0.0;
    public double winchFollowerMotorTemperature = 0.0;
  }

  /*
   * Updates the inputs
   */
  public default void updateInputs(WinchIOInputs inputs) {}

  @Override
  default void close() {}
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setClimberWinchPosition(double position, WinchIOInputs inputs) {}
  /*
   * Set voltage of the motor
   */
  public default void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {}

  public default void setClimberWinchPositionFast(double position, WinchIOInputs inputs) {}
}
