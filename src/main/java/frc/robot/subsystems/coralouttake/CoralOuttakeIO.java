package frc.robot.subsystems.coralouttake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralOuttakeIO extends AutoCloseable {

  @AutoLog
  public class CoralOuttakeIOInputs {
    public double tempCelcius = 0;
    public double appliedVoltage = 0;
    public boolean coralDetected = false;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(CoralOuttakeIOInputs inputs) {}

  public default void setPassiveMode(boolean passive) {}

  @Override
  default void close() {}
}
