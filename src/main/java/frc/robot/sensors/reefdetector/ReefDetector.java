package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.sensors.reefdetector.ReefDetectorIO.ReefDetectorIOInputs;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;

public class ReefDetector extends PeriodicBase {
  private ReefDetectorIO reefDetectorIO;
  private ReefDetectorIOInputs inputs = new ReefDetectorIOInputs();

  public ReefDetector(ReefDetectorIO reefDetectorIO) {
    this.reefDetectorIO = reefDetectorIO;
    AlertsManager.addAlert(
        () -> !inputs.isConnected, "Reef detector disconnected.", AlertType.kError);
  }

  @Override
  public void periodic() {
    reefDetectorIO.updateInputs(inputs);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public boolean isDetecting() {
    return inputs.isDetecting;
  }

  public double getDistanceToReef() {
    return inputs.distanceToReef;
  }

  public double getDeltaX() {
    return inputs.deltaX;
  }
}
