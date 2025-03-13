package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;
import org.littletonrobotics.junction.Logger;

public class ReefDetector extends PeriodicBase {
  private ReefDetectorIO reefDetectorIO;
  private ReefDetectorIOInputsAutoLogged inputs = new ReefDetectorIOInputsAutoLogged();
  private double foundThresh = ReefDetectorConstants.detectionThresh;

  public double getFoundThresh() {
    return foundThresh;
  }

  public ReefDetector(ReefDetectorIO reefDetectorIO) {
    this.reefDetectorIO = reefDetectorIO;
    AlertsManager.addAlert(
        () -> !inputs.isConnected, "Reef detector disconnected.", AlertType.kError);
  }

  @Override
  public void periodic() {
    reefDetectorIO.updateInputs(inputs);
    Logger.processInputs("ReefDetector", inputs);
    Logger.recordOutput("Found threshold", foundThresh);
    if (getDistanceToReef() < Double.MAX_VALUE) {
      Logger.recordOutput("FilteredDist", getDistanceToReef());
    }
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

  public void reefFind() {
    if (getDistanceToReef() < foundThresh) {
      foundThresh = getDistanceToReef();
    }
  }

  public void reefFindReset() {
    foundThresh = ReefDetectorConstants.detectionThresh;
  }
}
