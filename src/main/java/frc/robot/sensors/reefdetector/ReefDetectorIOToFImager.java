package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;

public class ReefDetectorIOToFImager implements ReefDetectorIO {

  private final DutyCycle ToFImagerDutyCycle;

  public ReefDetectorIOToFImager() {
    ToFImagerDutyCycle = new DutyCycle(new DigitalInput(ReefDetectorConstants.sensorTOFChannel));
  }

  /**
   * Returns between 0 - 255, 0 being not detecting, and the more up, the more right it is detecting
   *
   * @return
   */
  public double getRawValue() {
    return ToFImagerDutyCycle.getOutput() * 255.0;
  }

  public int getValue() {return (((int) Math.round(getValue() + 1)) / 16 - 1) / 2}

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = getValue() != 0;
    inputs.isDetecting = getValue() != 255 && inputs.isConnected;
    inputs.distanceToReef = 0.0;
    inputs.deltaX = getValue();
  }
}
