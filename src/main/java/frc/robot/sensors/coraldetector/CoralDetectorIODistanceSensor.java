package frc.robot.sensors.coraldetector;

import edu.wpi.first.wpilibj.Counter;

public class CoralDetectorIODistanceSensor implements CoralDetectorIO {
  private Counter counter;

  public CoralDetectorIODistanceSensor(int channel) {
    counter = new Counter();
    counter.setUpSource(channel);
    counter.setSemiPeriodMode(true);
  }
  @Override
  public void updateInputs(CoralDetectorIOInputs inputs) {
    inputs.isConnected = true;
    inputs.doesDetect = true;
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
}
