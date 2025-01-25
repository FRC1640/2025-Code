package frc.robot.sensors.reefdetector;

import java.util.function.Supplier;

public class ReefDetectorIOSim implements ReefDetectorIO {
  private Supplier<Double> distanceToReefSupplier;
  private Supplier<Double> deltaXSupplier;

  public ReefDetectorIOSim(
      Supplier<Double> distanceToReefSupplier, Supplier<Double> deltaXSupplier) {
    this.distanceToReefSupplier = distanceToReefSupplier;
    this.deltaXSupplier = deltaXSupplier;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.distanceToReef = distanceToReefSupplier.get();
    inputs.deltaX = deltaXSupplier.get();
  }
}
