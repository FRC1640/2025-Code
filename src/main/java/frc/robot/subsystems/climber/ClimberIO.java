package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.ClimberConstants;

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
  /**
   * Modifies the inputted voltage so as to not move out of limits
   *
   * @param pos the current position.
   * @param voltage the base voltage to clamp.
   * @return clamped voltage.
   */
  public default double applyLimits(double pos, double voltage) {
    double voltageClamped = voltage;
    if (!(Double.isNaN(voltageClamped) || Double.isNaN(pos))) {
      if (pos < ClimberConstants.liftMin) {
        voltageClamped = Math.max(voltage, 0);
      }
      if (pos > ClimberConstants.liftMax) {
        voltageClamped = Math.min(voltage, 0);
      }
    } else {
      return 0;
    }

    return voltageClamped;
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
  public default void setClimberLiftPosition(double position, ClimberIOInputs inputs) {}
  /*
   * Set voltage of the motor
   */
  public default void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {}
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setClimberWinchPosition(double position, ClimberIOInputs inputs) {}
  /*
   * Set voltage of the motor
   */
  public default void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {}
}
