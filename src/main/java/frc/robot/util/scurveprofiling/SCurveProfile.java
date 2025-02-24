package frc.robot.util.scurveprofiling;

import edu.wpi.first.wpilibj.Timer;

/** A class for generating smooth S-curve motion profiles with controlled jerk. */
public class SCurveProfile {
  private Constraints m_constraints;
  private State m_initial;
  private double m_initialTime;
  private int m_direction;

  public static class Constraints {
    public final double maxVelocity;
    public final double maxAcceleration;
    public final double maxJerk;

    public Constraints(double maxVelocity, double maxAcceleration, double maxJerk) {
      this.maxVelocity = maxVelocity;
      this.maxAcceleration = maxAcceleration;
      this.maxJerk = maxJerk;
      // MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);
    }
  }

  public static class State {
    public double position;
    public double velocity;

    public State() {}

    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  public SCurveProfile(Constraints constraints, State initial) {
    m_constraints = constraints;
    m_initial = initial;
    m_initialTime = Timer.getFPGATimestamp();
  }

  public State calculate(State current, State goal) {
    double t = Timer.getFPGATimestamp() - m_initialTime;

    m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
    goal = direct(goal);

    double D = goal.position - m_initial.position;
    double T_vel = 1.875 * D / m_constraints.maxVelocity;
    double T_acc = Math.sqrt(5.778 * D / m_constraints.maxAcceleration);
    double T_jerk = Math.cbrt(60 * D / (0.8 * m_constraints.maxJerk));
    double T = Math.max(Math.max(T_vel, T_acc), T_jerk);

    double position =
        m_initial.position
            + D
                * (10 * Math.pow((t / T), 3)
                    - 15 * Math.pow((t / T), 4)
                    + 6 * Math.pow((t / T), 5));
    double velocity =
        D
            * (30 * Math.pow(t, 2) / Math.pow(T, 3)
                - 60 * Math.pow(t, 3) / Math.pow(T, 4)
                + 30 * Math.pow(t, 4) / Math.pow(T, 5));

    State result = new State(position, velocity);
    return direct(result);
  }

  private static boolean shouldFlipAcceleration(State initial, State goal) {
    return initial.position > goal.position;
  }

  private State direct(State in) {
    return new State(in.position * m_direction, in.velocity * m_direction);
  }
}
