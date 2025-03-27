package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.Counter;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;

public class ReefDetectorIOTOFSensor implements ReefDetectorIO {
  public Counter sensor;
  
  public ReefDetectorIOTOFSensor() {
    sensor = new Counter(ReefDetectorConstants.sensorTOFChannel);
  }
  /**
   * Returns between 0 - 255, 0 being not detecting, and the more up, the more right it is detecting
   * @return
   */
  public double getValue(){
    double pulseWidth = sensor.getPeriod();
    return pulseWidth;
  }
  public boolean isReefDetected(){
    return getValue() != 255 && getValue() != 0;
  }
  
  public double getLocation(){
    // TODO 
    return 0.0;
  }
  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.distanceToReef = 0.0;
    inputs.deltaX = 0.0;
  }
}
