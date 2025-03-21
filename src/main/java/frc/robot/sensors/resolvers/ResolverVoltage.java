package frc.robot.sensors.resolvers;

import edu.wpi.first.wpilibj.AnalogInput;

public class ResolverVoltage {

  private AnalogInput resolver;
  private double slope;
  private double offset;

  /**
   * Creates resolver using point slope form (2 voltages and 2 angles in degrees)
   *
   * @param channel
   * @param v1
   * @param v2
   * @param angle1
   * @param angle2
   */
  public ResolverVoltage(int channel, double v1, double v2, double angle1, double angle2) {
    resolver = new AnalogInput(channel);
    slope = (angle1 - angle2) / (v1 - v2);
    offset = angle1 - slope * v1;
  }

  /**
   * Creates resolver using a slope (created from 2 voltages and degrees) and an offset. Ideal for
   * systems with same sloped resolvers but multiple offsets (like swerve)
   *
   * @param channel
   * @param v1
   * @param v2
   * @param angle1
   * @param angle2
   * @param offset
   */
  public ResolverVoltage(
      int channel, double v1, double v2, double angle1, double angle2, double offset) {
    resolver = new AnalogInput(channel);
    slope = (angle1 - angle2) / (v1 - v2);
    this.offset = offset;
  }

  /*
   * Creates resolver using ResolverVoltageInfo class
   */
  public ResolverVoltage(ResolverVoltageInfo info) {
    resolver = new AnalogInput(info.channel);
    slope = (info.angle1 - info.angle2) / (info.v1 - info.v2);
    if (info.offset == null) {
      offset = info.angle1 - slope * info.v1;
    } else {
      offset = info.offset;
    }
  }

  /**
   * @return Angle in radians
   */
  public double getRadians() {
    return Math.toRadians(getDegrees());
  }
  /**
   * @return Current voltage of encoder.
   */
  public double getVoltage() {
    return resolver.getVoltage();
  }
  /**
   * @return Angle in degrees
   */
  public double getDegrees() {
    double v = getVoltage();
    double angle = slope * v + offset;
    return angle;
  }

  public int getRawValue() {
    return resolver.getValue();
  }
}
