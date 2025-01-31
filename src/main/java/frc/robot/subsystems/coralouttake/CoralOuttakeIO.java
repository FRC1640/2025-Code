package frc.robot.subsystems.coralouttake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralOuttakeIO extends AutoCloseable {

  @AutoLog
  public class CoralOuttakeIOInputs {
    public double tempCelcius;
    public double appliedVoltage;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(CoralOuttakeIOInputs inputs) {}

  @Override
  default void close() {}
}
