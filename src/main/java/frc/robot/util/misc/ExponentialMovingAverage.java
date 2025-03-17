package frc.robot.util.misc;

public class ExponentialMovingAverage {
  // https://www.investopedia.com/terms/e/ema.asp

  private double multiplier;
  private double current;

  public ExponentialMovingAverage(double smoothing, double period) {
    this.multiplier = smoothing / (1 + period);
  }

  public void update(double data) {
    current = multiplier * data + (multiplier * current);
  }

  public double get() {
    return current;
  }
}
