package frc.robot.sensors.coraldetector;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.RobotConstants.CoralDetectorConstants;

public class CoralDetectorIOPixy implements CoralDetectorIO {
  private final AnalogInput pixyCamInput;

  public CoralDetectorIOPixy() {
    pixyCamInput = new AnalogInput(CoralDetectorConstants.channel);
  }

  @Override
  public void updateInputs(CoralDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.doesDetect = true;
    inputs.deltaX = pixyCamInput.getVoltage();
  }
}
