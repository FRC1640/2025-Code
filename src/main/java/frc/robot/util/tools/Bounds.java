package frc.robot.util.tools;

public class Bounds {
  public final Limit low;
  public final Limit high;

  /*
   * Create a new Idea Parameter
   */
  public Bounds(Limit low, Limit high) {
    this.low = low;
    this.high = high;
  }

  public Bounds(double low, double high) {
    this.low = new Limit(low);
    this.high = new Limit(high);
  }

  public Bounds(boolean low, boolean high) {
    this.low = new Limit(low);
    this.high = new Limit(high);
  }

  public Bounds(double low, boolean high) {
    this.low = new Limit(low);
    this.high = new Limit(high);
  }

  public Bounds(boolean low, double high) {
    this.low = new Limit(low);
    this.high = new Limit(high);
  }

  public String toString() {
    return "Low: " + low.toString() + " High: " + high.toString();
  }
}
