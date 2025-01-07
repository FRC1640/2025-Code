package frc.robot.util.control;

public class FeedForwardConstants {
  public final double kS;
  public final double kV;
  public final double kA;

  public FeedForwardConstants(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
