package frc.robot.sensors.reefdetector;

import frc.robot.sensors.reefdetector.ReefDetectorIO.ReefDetectorIOInputs;
import frc.robot.util.periodic.PeriodicBase;

public class ReefDetector extends PeriodicBase {
  private ReefDetectorIO coralDetectorIO;
  private ReefDetectorIOInputs inputs = new ReefDetectorIOInputs();

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
