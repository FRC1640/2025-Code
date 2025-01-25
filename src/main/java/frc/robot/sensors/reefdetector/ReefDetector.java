package frc.robot.sensors.reefdetector;

import frc.robot.sensors.reefdetector.ReefDetectorIO.CoralDetectorIOInputs;
import frc.robot.util.periodic.PeriodicBase;

public class ReefDetector extends PeriodicBase {
  private ReefDetectorIO coralDetectorIO;
  private CoralDetectorIOInputs inputs = new CoralDetectorIOInputs();

  public ReefDetector(ReefDetectorIO coralDetectorIO) {
    this.coralDetectorIO = coralDetectorIO;
  }

  @Override
  public void periodic() {
    coralDetectorIO.updateInputs(inputs);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public double getDistanceToReef() {
    return inputs.distanceToReef;
  }

  public double getDeltaX() {
    return inputs.deltaX;
  }
}
