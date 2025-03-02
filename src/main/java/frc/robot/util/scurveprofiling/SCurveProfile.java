package frc.robot.util.scurveprofiling;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SCurveProfile extends TrapezoidProfile {
  private Constraints m_constraints;
  private double m_targetPosition;
  private double m_startPosition;
  private double m_startVelocity;
  private int m_direction;
  private double m_profileJerk;

  // Phase tracking
  private ProfilePhase m_currentPhase;
  private double m_currentVelocity;
  private double m_currentPosition;

  // Precomputed timing values
  private double m_currentAcceleration;
  private double m_initialPosition;
  private double m_quarterTime;

  enum ProfilePhase {
    ACCEL_JERK_UP,
    ACCEL_JERK_DOWN,
    DECEL_JERK_DOWN,
    DECEL_JERK_UP,
    FINISHED
  }

  public static class Constraints extends TrapezoidProfile.Constraints {
    public final double maxJerk;

    public Constraints(double maxVelocity, double maxAcceleration, double maxJerk) {
      super(maxVelocity, maxAcceleration);
      this.maxJerk = Math.abs(maxJerk * 10.0);
    }
  }

  public SCurveProfile(Constraints constraints) {
    super(constraints);
    m_constraints = constraints;
  }

  public void initialize(double currentPosition, double currentVelocity, double targetPosition) {
    m_direction = (targetPosition >= currentPosition) ? 1 : -1;
    m_initialPosition = currentPosition;
    m_targetPosition = targetPosition;

    m_currentPosition = currentPosition;
    m_currentVelocity = currentVelocity;
    m_currentAcceleration = 0;

    final double J = m_constraints.maxJerk;
    final double A = m_constraints.maxAcceleration;
    final double V = m_constraints.maxVelocity;
    final double D = Math.abs(targetPosition - currentPosition);

    m_quarterTime =
        Math.min(
            ((Math.PI * A) / (2 * J)),
            Math.min(Math.sqrt((Math.PI * V) / (2 * A)), (Math.cbrt((Math.PI * D) / (2 * J)))));

    m_currentPhase = ProfilePhase.ACCEL_JERK_UP;
  }

  public State calculate(double deltaT) {
    switch (m_currentPhase) {
      case ACCEL_JERK_UP:
        m_profileJerk = m_constraints.maxJerk;
        if (m_currentAcceleration >= m_constraints.maxAcceleration) {
          m_currentAcceleration = m_constraints.maxAcceleration;
          m_currentPhase = ProfilePhase.ACCEL_JERK_DOWN;
          System.out.println("ACCEL_JD");
        }
      case ACCEL_JERK_DOWN:
        m_profileJerk = -m_constraints.maxJerk;
        if (m_currentAcceleration < 0) {
          m_currentVelocity = m_constraints.maxVelocity;
          m_currentPhase = ProfilePhase.DECEL_JERK_DOWN;
          System.out.println("DECEL_JD");
        }
      case DECEL_JERK_DOWN:
        m_profileJerk = -m_constraints.maxJerk;
        if (m_currentAcceleration <= -m_constraints.maxAcceleration) {
          m_currentAcceleration = -m_constraints.maxAcceleration;
          m_currentPhase = ProfilePhase.DECEL_JERK_UP;
          System.out.println("DECEL_CONSTANT");
        }
      case DECEL_JERK_UP:
        m_profileJerk = m_constraints.maxJerk;
        if (m_currentVelocity <= 0) {
          m_currentVelocity = 0;
          m_currentPhase = ProfilePhase.FINISHED;
          System.out.println("FINISHED");
        }
      case FINISHED:
        m_profileJerk = 0;
        m_currentAcceleration = 0;
        m_currentVelocity = 0;
        break;
      default:
        break;
    }
    update(deltaT); // updates the m_currentPosition, m_currentVelocity, and m_currentAcceleration
    return new State(m_currentPosition, m_currentVelocity);
  }

  private void update(double Dt) {
    m_currentAcceleration += m_profileJerk * Dt;
    m_currentVelocity += m_currentAcceleration * Dt;
    m_currentPosition += m_currentVelocity * Dt;
  }
}
