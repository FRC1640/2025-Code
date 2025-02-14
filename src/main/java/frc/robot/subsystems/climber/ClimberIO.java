package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public static class ClimberIOInputs {
    // may need more for pneumatic stsuff not entirely sure
    public double liftMotorPosition = 0.0;
    public double liftMotorVelocity = 0.0;
    public double liftMotorCurrent = 0.0;
    public double liftMotorVoltage = 0.0;
    public double liftMotorTemperature = 0.0;

    public double winchLeaderMotorPosition = 0.0;
    public double winchFollowerMotorPosition = 0.0;
    public double winchLeaderMotorVelocity = 0.0;
    public double winchFollowerMotorVelocity = 0.0;
    public double winchLeaderMotorCurrent = 0.0;
    public double winchFollowerMotorCurrent = 0.0;
    public double winchLeaderMotorVoltage = 0.0;
    public double winchFollowerMotorVoltage = 0.0;
    public double winchLeaderMotorTemperature = 0.0;
    public double winchFollowerMotorTemperature = 0.0;

    public boolean solenoidForward = false;
  }

  /*
   * Updates the inputs
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  @Override
  default void close() {}
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
  /*
   * Set solenoid state (forward/reverse)
   */
  public default void setSolenoidState(boolean forward, ClimberIOInputs inputs) {}

}
