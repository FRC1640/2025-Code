package frc.robot.subsystems.gantry;

import org.littletonrobotics.junction.AutoLog;

public interface GantryIO extends AutoCloseable {
  @AutoLog
  public class GantryIOInputs {
    public int rawEncoderValue;
    public double carriageEncoderVoltage;
    public double carriageEncoderRelative;

    public double carriageTempCelcius;
    public double carriageAppliedVoltage;
    public double driveCurrentAmps;
  }

  public default void updateInputs(GantryIOInputs inputs) {}

  public default void setGantryVoltage(double voltage) {}

  public default void setCarriagePosition(int pos) {} // 0/1/2 to indicate left/center/right

  @Override
  default void close() {}
}
