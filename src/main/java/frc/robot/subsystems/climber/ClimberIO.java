package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
// import frc.robot.constants.RobotConstants.ClimberConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public static class ClimberIOInputs {
    // may need more for pneumatic stuff not entirely sure
    public double liftMotorPosition = 0.0;
    public double winch1MotorPosition = 0.0;
    public double winch2MotorPosition = 0.0;
    public double liftMotorVelocity = 0.0;
    public double winch1MotorVelocity = 0.0;
    public double winch2MotorVelocity = 0.0;
    public double liftMotorCurrent = 0.0;
    public double winch1MotorCurrent = 0.0;
    public double winch2MotorCurrent = 0.0;
    public double liftMotorVoltage = 0.0;
    public double winch1MotorVoltage = 0.0;
    public double winch2MotorVoltage = 0.0;
    public double liftTemperature = 0.0;
    public double winch1Temperature = 0.0;
    public double winch2Temperature = 0.0;
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
