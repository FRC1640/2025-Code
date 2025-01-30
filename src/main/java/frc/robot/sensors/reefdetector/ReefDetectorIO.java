package frc.robot.sensors.reefdetector;

import org.littletonrobotics.junction.AutoLog;

public interface ReefDetectorIO {
  @AutoLog
  public static class ReefDetectorIOInputs {
    public boolean isConnected = false;
    public boolean isDetecting = false;
    public double distanceToReef = 0.0;
    public double deltaX = 0.0;
  }

  public default void updateInputs(ReefDetectorIOInputs inputs) {}
}
