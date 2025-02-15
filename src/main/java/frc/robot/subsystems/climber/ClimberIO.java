package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public static class ClimberIOInputs {
    public double elevatorMotorPosition = 0.0;
    public double elevatorMotorVelocity = 0.0;
    public double elevatorMotorCurrent = 0.0;
    public double elevatorMotorVoltage = 0.0;
    public double elevatorMotorTemperature = 0.0;

    public boolean solenoidForward = false;
    public boolean sensor1 = false;
    public boolean sensor2 = false;
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
   * Set solenoid state (forward/reverse)
   */
  public default void setSolenoidState(boolean forward) {}
  /*
   * Set servo position (between 0 and 1)
   */
  public default void setServoPosition(double position) {}
}
