package frc.robot.subsystems.algaeintake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO extends AutoCloseable {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    // may need more for pneumatic stsuff not entirely sure
    public double intakeMotorPosition = 0.0;
    public double intakeMotorVelocity = 0.0;
    public double intakeMotorCurrent = 0.0;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorTemperature = 0.0;
    public boolean solenoidForward = false;

    public void setSolenoidState(final boolean forward, AlgaeIntakeIOSim intakeIOSim) {
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
  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  @Override
  default void close() {}
  /*
   * Sets the position of the motor(s) using a PID
   */
  public default void setIntakeMotorPosition(double position, AlgaeIntakeIOInputs inputs) {}
  /*
   * Set voltage of the motor
   */
  public default void setIntakeMotorVoltage(double voltage, AlgaeIntakeIOInputs inputs) {}
  /*
   * Set solenoid state (forward/reverse)
   * Boolean forward is reverse
   */
  public default void setIntakeSolenoidState(boolean forward, AlgaeIntakeIOInputs inputs) {}
}
