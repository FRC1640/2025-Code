package frc.robot.sensors.coraldetector;

import java.util.function.Supplier;

public class ReefDetectorIOSim implements ReefDetectorIO {
  private Supplier<Double> deltaXSupplier;

  public ReefDetectorIOSim(Supplier<Double> deltaXSupplier) {
    this.deltaXSupplier = deltaXSupplier;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.doesDetect = true;
    inputs.deltaX = deltaXSupplier.get();
  }
}
