package frc.robot.subsystems.gantry;

import org.littletonrobotics.junction.AutoLog;

public interface GantryIO extends AutoCloseable {
  @AutoLog
  public class GantryIOInputs {
    public double encoderPosition;
    public double encoderVoltage;

    public double tempCelcius;
    public double appliedVoltage;
    public double currentAmps;
  }

  public default void updateInputs(GantryIOInputs inputs) {}

  public default void setGantryVoltage(double voltage) {}

  public default void setCarriagePosition(int pos) {} // 0/1/2 to indicate left/center/right

  @Override
  default void close() {}
}
