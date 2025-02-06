package frc.robot.sensors.resolvers;

public class ResolverVoltageInfo {
  int channel;
  double v1;
  double v2;
  double angle1;
  double angle2;
  Double offset;

  /*
   * Leave offset null if no special offset needed
   */
  public ResolverVoltageInfo(
      int channel, double v1, double v2, double angle1, double angle2, Double offset) {
    this.channel = channel;
    this.v1 = v1;
    this.v2 = v2;
    this.angle1 = angle1;
    this.angle2 = angle2;
    this.offset = offset;
  }
}
