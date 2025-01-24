package frc.robot.sensors.distance;

import edu.wpi.first.wpilibj.Counter;

public class PwmDistanceSensor {
  private Counter counter;

  public PwmDistanceSensor(int channel) {
    counter = new Counter();
    counter.setUpSource(channel);
    counter.setSemiPeriodMode(true);
  }

  /**
   * Gets distance converted from PWM reading.
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
      return Math.max(0, 2 * (width - 1000));
    }
  }
}
