package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {
  @AutoLog
  public static class IntakeIOInputs {
    // may need more for pneumatic stsuff not entirely sure
    public double intakeMotorPosition = 0.0;
    public double intakeMotorVelocity = 0.0;
    public double intakeMotorCurrent = 0.0;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorTemperature = 0.0;
    public boolean solenoidForward = false;

    public void setSolenoidState(final boolean forward, IntakeIOSim intakeIOSim) {
      if (forward) {
        intakeIOSim.doubleSolenoidSim.set(DoubleSolenoid.Value.kForward);
      } else {
        intakeIOSim.doubleSolenoidSim.set(DoubleSolenoid.Value.kReverse);
      }
    }
  }

  /*
   * Updates the inputs
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  @Override
  default void close() {}
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setIntakemotor1Position(double position, IntakeIOInputs inputs) {}
  /*
   * Set voltage of the motor
   */
  public default void setIntakemotor1Voltage(double voltage, IntakeIOInputs inputs) {}
  /*
   * Set solenoid state (forward/reverse)
   * Boolean forward is reverse
   */
  public default void setIntakeSolenoidState(boolean forward, IntakeIOInputs inputs) {}
}
