package frc.robot.sensors.reefdetector;

import edu.wpi.first.wpilibj.Counter;

public class ReefDetectorIODistanceSensor implements ReefDetectorIO {
  private Counter counter;

  public ReefDetectorIODistanceSensor(int channel) {
    counter = new Counter();
    counter.setUpSource(channel);
    counter.setSemiPeriodMode(true);
  }

  /**
   * Gets distance in millimeters converted from PWM reading.
   *
   * @return Distance.
   */
  public double getDistance() {
    double width = counter.getPeriod();
    if (width == 0) {
      System.out.println("timeout");
      return -1;
    } else if (width > 1850) {
      return -1;
    } else {
      return (2.0) * ((width * 1000000.0) - 10000.0);
    }
  }

  @Override
  public void updateInputs(CoralDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.distanceToReef = getDistance();
    inputs.deltaX = Integer.MIN_VALUE;
  }
}
