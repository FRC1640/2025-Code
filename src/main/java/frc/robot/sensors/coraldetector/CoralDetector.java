package frc.robot.sensors.coraldetector;

import frc.robot.sensors.coraldetector.CoralDetectorIO.CoralDetectorIOInputs;
import frc.robot.util.periodic.PeriodicBase;

public class CoralDetector extends PeriodicBase {
  private CoralDetectorIO coralDetectorIO;
  private CoralDetectorIOInputs inputs = new CoralDetectorIOInputs();

  public CoralDetector(CoralDetectorIO coralDetectorIO) {
    this.coralDetectorIO = coralDetectorIO;
  }

  @Override
  public void periodic() {
    coralDetectorIO.updateInputs(inputs);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public boolean doesDetect() {
    return inputs.doesDetect;
  }

  public double getDeltaX() {
    return inputs.deltaX;
  }
}
