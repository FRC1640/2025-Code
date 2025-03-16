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
    public double motorPosition = 0.0;

    public boolean isLimitSwitchPressed = false;
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

  public default void setLiftPositionMotionProfile(double position, LiftIOInputs inputs) {}

  public default void resetLiftMotionProfile(LiftIOInputs inputs) {}

  public default void resetEncoder() {}

  public default void resetLiftPositionPid() {}

  public default double velocitySetpoint() {
    return 0.0;
  }

  public default void testMethod() {}

  public default void setLimitEnabled(boolean enable) {}

  public default void updateEMA(double data) {}
}
