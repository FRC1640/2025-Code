package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;

public class ReefDetectorIOToFImager implements ReefDetectorIO {

  private final DutyCycle ToFImagerDutyCycle;
  private final DigitalInput ToFImagerDigitalInput;
  private boolean lastReading;

  public ReefDetectorIOToFImager() {
    ToFImagerDigitalInput = new DigitalInput(ReefDetectorConstants.sensorTOFChannel);
    ToFImagerDutyCycle = new DutyCycle(ToFImagerDigitalInput);
  }

  /**
   * Raw Value Duty Cycle Ratio
   *
   * @return Returns between 0 - 1, 0 being not detecting, and the more up, the more right it is
   *     detecting
   */
  public double getRawValue() {
    return ToFImagerDigitalInput.get() && ToFImagerDutyCycle.getOutput() == 0
        ? 1
        : ToFImagerDutyCycle.getOutput();
  }

  /**
   * Column being triggered on the reef detector
   *
   * @return Returns 0-7 although currently we have disabled it from 0 and 7 so... 1-6 returns -1 if
   *     sensor is disconnected
   */
  public double getColumn() {
    return Math.round(getRawValue() * 255 / 10) * 10;
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = getRawValue() != 0;
    inputs.isDetecting = (getColumn() >= 40 && getColumn() <= 50);
    lastReading = inputs.isDetecting;
    inputs.distanceToReef = 0.0;
    inputs.deltaX = getColumn();
  }
}
