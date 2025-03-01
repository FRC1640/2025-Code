package frc.robot.sensors.reefdetector;

import java.util.function.Supplier;

public class ReefDetectorIOSim implements ReefDetectorIO {
  private Supplier<Boolean> isReef;

  public ReefDetectorIOSim(Supplier<Boolean> isReef) {
    this.isReef = isReef;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.isDetecting = isReef.get();
  }
}
