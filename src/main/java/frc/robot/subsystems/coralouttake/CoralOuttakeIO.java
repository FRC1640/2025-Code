package frc.robot.subsystems.coralouttake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralOuttakeIO extends AutoCloseable {

  @AutoLog
  public class CoralOuttakeIOInputs {
    public double tempCelcius = 0;
    public double appliedVoltage = 0;
    public double outtakeVelocity = 0;
    public boolean coralDetectedHigh = false;
    public boolean hasCoral = false;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void setIntakeVelocity(double velocity, CoralOuttakeIOInputs inputs) {}

  public default void updateInputs(CoralOuttakeIOInputs inputs) {}

  @Override
  default void close() {}
}
