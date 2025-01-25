package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {}

  public default void updateInputs(LiftIOInputs inputs) {}

  @Override
  default void close() {}
}
