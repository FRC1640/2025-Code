package frc.robot.sensors.reefdetector;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ReefDetectorIOSim implements ReefDetectorIO {
  private Supplier<Double> distanceToReefSupplier;
  private Supplier<Double> deltaXSupplier;
  private BooleanSupplier detectedSupplier;

  public ReefDetectorIOSim(
      Supplier<Double> distanceToReefSupplier,
      Supplier<Double> deltaXSupplier,
      BooleanSupplier detectedSupplier) {
    this.distanceToReefSupplier = distanceToReefSupplier;
    this.deltaXSupplier = deltaXSupplier;
    this.detectedSupplier = detectedSupplier;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.isDetecting = distanceToReefSupplier.get() < 250 || detectedSupplier.getAsBoolean();
    inputs.distanceToReef = distanceToReefSupplier.get();
    inputs.deltaX = deltaXSupplier.get();
  }
}
