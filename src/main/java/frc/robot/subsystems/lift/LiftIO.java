package frc.robot.subsystems.lift;

import edu.wpi.first.math.MathUtil;
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
    public double leaderTemperature = 0.0;
    public double followerTemperature = 0.0;
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
   * Applies limits from the max and min of the motors
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
   * Clamps voltage between -12 and 12
   */
  public default double clampVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    if (Math.abs(voltage) < 0.001) {
      voltage = 0;
    }
    return voltage;
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setLiftPosition(double position, LiftIOInputs inputs) {}
  /*
   * Get Inputs
   */
  public default LiftIOInputs getInputs() {
    return new LiftIOInputs();
  }
}
