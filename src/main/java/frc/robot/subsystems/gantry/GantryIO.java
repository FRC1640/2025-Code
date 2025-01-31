package frc.robot.subsystems.gantry;

import org.littletonrobotics.junction.AutoLog;

public interface GantryIO extends AutoCloseable {
  @AutoLog
  public class GantryIOInputs {
    public double encoderPosition;
    public double tempCelcius;
    public double appliedVoltage;
    public double currentAmps;
  }

  public default void updateInputs(GantryIOInputs inputs) {}

  public default void setGantryVoltage(double voltage, GantryIOInputs inputs) {}

  public default void setGantryPosition(double pos, GantryIOInputs inputs) {}

  @Override
  default void close() {}
}
