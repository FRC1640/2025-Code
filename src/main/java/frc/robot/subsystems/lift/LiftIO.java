package frc.robot.subsystems.lift;

import frc.robot.constants.RobotConstants.LiftConstants;
import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {
    public double liftmotor1Position = 0.0;
    public double liftmotor2Position = 0.0;
    public double liftmotor1Velocity = 0.0;
    public double liftmotor2Velocity = 0.0;
    public double liftmotor1Current = 0.0;
    public double liftmotor2Current = 0.0;
    public double liftmotor1Voltage = 0.0;
    public double liftmotor2Voltage = 0.0;
  }

  public default void updateInputs(LiftIOInputs inputs) {}

  @Override
  default void close() {}

  /*
   * Set voltage of the motor
   */
  public default void setVoltage(double voltage) {}

  /*
   * Set speed % between -1 and 1
   */
  public default void setSpeed(double speed) {}
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
}
