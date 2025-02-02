package frc.robot.util.tools;

public class Limit {
  public final Double value;
  public final Boolean condition;

  public Limit(double value) {
    this.value = value;
    this.condition = null;
  }

  public Limit(Boolean condition) {
    this.condition = condition;
    this.value = null;
  }

  public String toString() {
    return value != null ? value.toString() : condition.toString();
  }
}
