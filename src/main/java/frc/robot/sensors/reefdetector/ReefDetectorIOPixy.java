package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.RobotConstants.CoralDetectorConstants;

public class ReefDetectorIOPixy implements ReefDetectorIO {
  private final AnalogInput pixyCamInput;

  public ReefDetectorIOPixy() {
    pixyCamInput = new AnalogInput(CoralDetectorConstants.channel);
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.distanceToReef = 0.0;
    inputs.deltaX = pixyCamInput.getVoltage();
  }
}
