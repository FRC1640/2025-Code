package frc.robot.sensors.coraldetector;

import java.util.function.Supplier;

public class CoralDetectorIOSim implements CoralDetectorIO {
  private Supplier<Double> deltaXSupplier;

  public CoralDetectorIOSim(Supplier<Double> deltaXSupplier) {
    this.deltaXSupplier = deltaXSupplier;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.doesDetect = true;
    inputs.deltaX = deltaXSupplier.get();
  }
}
