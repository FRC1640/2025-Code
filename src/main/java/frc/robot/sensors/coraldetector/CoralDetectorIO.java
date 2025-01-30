package frc.robot.sensors.coraldetector;

import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectorIO {
  @AutoLog
  public static class ReefDetectorIOInputs {
    public boolean isConnected = false;
    public boolean doesDetect = false;
    public double deltaX = 0.0;
  }

  public default void updateInputs(ReefDetectorIOInputs inputs) {}
}
