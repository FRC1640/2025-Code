package frc.robot.util.scurveprofiling;

import java.util.Objects;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;

/**
 * A class for generating smooth S-curve motion profiles with controlled jerk.
 */
public class SCurveProfile {
  private int m_direction;
  private final Constraints m_constraints;
  private State m_current;
  private double m_endJerk;
  private double m_endAccel;
  private double m_endFullSpeed;
  private double m_endDecel;
  private double m_jerk;

  public static class Constraints {
    public final double maxVelocity;
    public final double maxAcceleration;
    public final double maxJerk;

    public Constraints(double maxVelocity, double maxAcceleration, double maxJerk) {
      this.maxVelocity = maxVelocity;
      this.maxAcceleration = maxAcceleration;
      this.maxJerk = maxJerk;
      MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);
    }
  }

  public static class State {
    public double position;
    public double velocity;
    public double acceleration;

    public State() {}

    public State(double position, double velocity, double acceleration) {
      this.position = position;
      this.velocity = velocity;
      this.acceleration = acceleration;
    }
  }

  public SCurveProfile(Constraints constraints) {
    m_constraints = constraints;
    m_jerk = constraints.maxJerk;
  }

  public State calculate(double t, State current, State goal) {
    m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
    m_current = direct(current);
    goal = direct(goal);

    double jerkTime = m_constraints.maxAcceleration / m_constraints.maxJerk;
    double accelTime = jerkTime + (m_constraints.maxVelocity / m_constraints.maxAcceleration);

    m_endJerk = jerkTime;
    m_endAccel = accelTime;
    m_endFullSpeed = m_endAccel + (goal.position - m_current.position) / m_constraints.maxVelocity;
    m_endDecel = m_endFullSpeed + accelTime;

    State result = new State(m_current.position, m_current.velocity, m_current.acceleration);

    if (t < m_endJerk) {
      result.acceleration += t * m_jerk;
      result.velocity += result.acceleration * t;
      result.position += result.velocity * t + 0.5 * result.acceleration * t * t;
    } else if (t < m_endAccel) {
      result.acceleration = m_constraints.maxAcceleration;
      result.velocity += result.acceleration * t;
      result.position += result.velocity * t;
    } else if (t < m_endFullSpeed) {
      result.acceleration = 0;
      result.velocity = m_constraints.maxVelocity;
      result.position += result.velocity * t;
    } else if (t < m_endDecel) {
      result.acceleration = -m_constraints.maxAcceleration;
      result.velocity += result.acceleration * t;
      result.position += result.velocity * t;
    } else {
      result = goal;
    }

    return direct(result);
  }

  private static boolean shouldFlipAcceleration(State initial, State goal) {
    return initial.position > goal.position;
  }

  private State direct(State in) {
    return new State(in.position * m_direction, in.velocity * m_direction, in.acceleration * m_direction);
  }
}
