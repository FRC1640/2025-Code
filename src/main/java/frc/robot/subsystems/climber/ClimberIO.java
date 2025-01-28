package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
// import frc.robot.constants.RobotConstants.ClimberConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public static class ClimberIOInputs {
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
  public default void updateInputs(ClimberIOInputs inputs) {}

  @Override
  default void close() {}

  /*
   * Set voltage of the motor
   */
  public default void setClimberVoltage(double voltage) {}
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
  public default void setClimberPosition(double position) {}
}
