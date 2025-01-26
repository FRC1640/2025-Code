package frc.robot.subsystems.lift;

import frc.robot.constants.RobotConstants.LiftConstants;
import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {
    public double leadermotorPosition = 0.0;
    public double followermotorPosition = 0.0;
    public double leadermotorVelocity = 0.0;
    public double followermotorVelocity = 0.0;
    public double leadermotorCurrent = 0.0;
    public double followermotorCurrent = 0.0;
    public double leadermotorVoltage = 0.0;
    public double followermotorVoltage = 0.0;
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
  public default void setVoltage(double voltage) {}
  /*
   * Clamp the speed of the motor to prevent it from going out of bounds
   */
  public default double clampSpeed(double motorPos, double motorSpeed) {
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
  public default void setPosition(double position) {}
}
