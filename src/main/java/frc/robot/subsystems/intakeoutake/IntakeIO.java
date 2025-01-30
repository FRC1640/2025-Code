package frc.robot.subsystems.intakeoutake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralaratorIO extends AutoCloseable {

  @AutoLog
  public class CoralaratorIOInputs {
    public double tempCelcius;
    public double appliedVoltage;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(CoralaratorIOInputs inputs) {}

  @Override
  default void close() {}
}
