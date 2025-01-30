package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {
    public double leaderMotorPosition = 0.0;
    public double followerMotorPosition = 0.0;
    public double leaderMotorVelocity = 0.0;
    public double followerMotorVelocity = 0.0;
    public double leaderMotorCurrent = 0.0;
    public double followerMotorCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double followerMotorVoltage = 0.0;
    public double leaderTemperature = 0.0;
    public double followerTemperature = 0.0;
    public double leaderAppliedVoltage = 0.0;
    public double followerAppliedVoltage = 0.0;
  }
  /*
   * Updates the inputs
   */
  public default void updateInputs(LiftIOInputs inputs) {}

  @Override
  default void close() {}

  /*
   * Set voltage of the motor
   */
  public default void setLiftVoltage(double voltage, LiftIOInputs inputs) {}
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setLiftPosition(double position, LiftIOInputs inputs) {}
}
