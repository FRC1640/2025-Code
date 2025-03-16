package frc.robot.util.misc;

public class EMA {

  private double multiplier;
  private double current;

  public EMA(double smoothing, double period) {
    this.multiplier = smoothing / (1 + period);
  }

  public void update(double data) {
    current = multiplier * data + (multiplier * current);
  }

  public double get() {
    return current;
  }
}
