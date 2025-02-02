package frc.robot.subsystems.gantry;

import org.littletonrobotics.junction.AutoLog;

public interface GantryIO extends AutoCloseable {
  @AutoLog
  public class GantryIOInputs {
    public double encoderPosition;
    public double tempCelcius;
    public double appliedVoltage;
    public double currentAmps;
    public boolean isLimitSwitchPressed;
    public double encoderVelocity;
  }

  public default void updateInputs(GantryIOInputs inputs) {}

  public default void setGantryVoltage(double voltage, GantryIOInputs inputs) {}

  public default void setGantryPosition(double pos, GantryIOInputs inputs) {}

  public default void resetEncoder() {}

  @Override
  default void close() {}
}
