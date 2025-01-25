package frc.robot.sensors.resolvers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ResolverPWM {

  private DutyCycleEncoder resolver;
  private double offset;
  private int channel;

  public ResolverPWM(int channel, double offset) {
    this.channel = channel;
    resolver = new DutyCycleEncoder(new DigitalInput(channel));
    this.offset = offset;
  }
  /**
   * @return Angle in radians
   */
  public double getRadians() {
    return Math.toRadians(getDegrees());
  }
  /**
   * @return Current frequency of encoder.
   */
  public double getFrequency() {
    return resolver.getFrequency();
  }
  /**
   * @return Angle in degrees
   */
  public double getDegrees() {
    // Logger.recordOutput("Pivot/" + channel, resolver.get());
    return ((resolver.get() * 360 - offset));
  }
}
