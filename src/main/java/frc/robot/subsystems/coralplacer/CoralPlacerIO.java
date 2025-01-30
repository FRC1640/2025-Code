package frc.robot.subsystems.coralplacer;

import org.littletonrobotics.junction.AutoLog;

public interface CoralPlacerIO extends AutoCloseable {

  @AutoLog
  public class CoralPlacerIOInputs {
    public double tempCelcius;
    public double appliedVoltage;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(CoralPlacerIOInputs inputs) {}

  @Override
  default void close() {}
}
