package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO extends AutoCloseable {

  @AutoLog
  public static class WinchIOInputs {
    public double winch1MotorPosition = 0.0;
    public double winch2MotorPosition = 0.0;
    public double winch1MotorVelocity = 0.0;
    public double winch2MotorVelocity = 0.0;
    public double winch1MotorCurrent = 0.0;
    public double winch2MotorCurrent = 0.0;
    public double winch1MotorVoltage = 0.0;
    public double winch2MotorVoltage = 0.0;
    public double winch1MotorTemperature = 0.0;
    public double winch2MotorTemperature = 0.0;
    public double winchAngle = 0.0;
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
   * Sets the angle of the winch motors using a PID
   * angle counts up clockwise starting at 0 degrees = due west
   */
  public default void setClimberWinchAngle(double angle, WinchIOInputs inputs) {}
  /*
   * Set voltage of both motors
   */
  public default void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {}
  /*
   * Set voltage of motor 1
   */
  public default void setClimberWinch1Voltage(double voltage, WinchIOInputs inputs) {}
  /*
   * Set voltage of motor 2
   */
  public default void setClimberWinch2Voltage(double voltage, WinchIOInputs inputs) {}
}
