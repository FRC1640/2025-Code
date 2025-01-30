package frc.robot.sensors.coraldetector;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.sensors.coraldetector.CoralDetectorIO.ReefDetectorIOInputs;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;

public class CoralDetector extends PeriodicBase {
  private CoralDetectorIO coralDetectorIO;
  private ReefDetectorIOInputs inputs = new ReefDetectorIOInputs();

  public CoralDetector(CoralDetectorIO coralDetectorIO) {
    this.coralDetectorIO = coralDetectorIO;
  }

  @Override
  public void periodic() {
    coralDetectorIO.updateInputs(inputs);
    AlertsManager.addAlert(
        () -> !inputs.isConnected, "Coral detector disconnected.", AlertType.kError);
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
