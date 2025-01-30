package frc.robot.subsystems.intakeoutake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {

  @AutoLog
  public class IntakeIOInputs {
    public double tempCelcius;
    public double appliedVoltage;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  @Override
  default void close() {}
}
