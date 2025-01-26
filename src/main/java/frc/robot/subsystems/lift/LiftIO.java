package frc.robot.subsystems.lift;

import frc.robot.constants.RobotConstants.LiftConstants;
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
  public default void setLiftVoltage(double voltage) {}
  /*
   * Clamp the speed of the motor to prevent it from going out of bounds
   */
  public default double applyLimits(double motorPos, double motorSpeed) {
    if (motorPos > LiftConstants.liftMax) {
      return Math.min(0, motorSpeed);
    } else if (motorPos < LiftConstants.liftMin) {
      return Math.max(0, motorSpeed);
    }
    return motorSpeed;
  }

  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setLiftPosition(double position) {}
}
