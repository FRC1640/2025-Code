package frc.robot.sensors.coraldetector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.sensors.coraldetector.CoralDetectorIO.CoralDetectorIOInputs;
import frc.robot.util.periodic.PeriodicBase;

public class CoralDetector extends PeriodicBase {
  private CoralDetectorIO coralDetectorIO;
  private CoralDetectorIOInputs inputs = new CoralDetectorIOInputs();
  private final Alert coralDetectorDisconnectedAlert =
      new Alert("Coral detector disconnected.", AlertType.kError);

  public CoralDetector(CoralDetectorIO coralDetectorIO) {
    this.coralDetectorIO = coralDetectorIO;
  }

  @Override
  public void periodic() {
    coralDetectorIO.updateInputs(inputs);
    coralDetectorDisconnectedAlert.set(!inputs.isConnected);
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
