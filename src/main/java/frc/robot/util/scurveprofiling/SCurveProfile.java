package frc.robot.util.scurveprofiling;

public class SCurveProfile {
  private Constraints m_constraints;
  private double m_targetPosition;
  private double m_startPosition;
  private double m_startVelocity;

  // Phase tracking
  private double m_elapsedTime;
  private ProfilePhase m_currentPhase;
  private double m_currentVelocity;
  private double m_currentPosition;

  // Precomputed timing values
  private double m_t1, m_t2, m_t3, m_t4, m_t5, m_t6, m_totalTime;
  private double m_cruiseDistance;

  enum ProfilePhase {
    ACCEL_JERK_UP,
    ACCEL_CONSTANT,
    ACCEL_JERK_DOWN,
    CRUISE,
    DECEL_JERK_DOWN,
    DECEL_CONSTANT,
    DECEL_JERK_UP,
    FINISHED
  }

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

  public SCurveProfile(Constraints constraints) {
    m_constraints = constraints;
  }

  public void initialize(double currentPosition, double currentVelocity, double targetPosition) {
    m_startPosition = currentPosition;
    m_startVelocity = currentVelocity;
    m_targetPosition = targetPosition;
    m_elapsedTime = 0;
    m_currentPosition = currentPosition;
    m_currentVelocity = currentVelocity;

    // Calculate phase timings
    final double J = m_constraints.maxJerk;
    final double A = m_constraints.maxAcceleration;
    final double V = m_constraints.maxVelocity;

    // Time to reach max acceleration (jerk up phase)
    m_t1 = Math.min(A / J, Math.sqrt((V - m_startVelocity) / J));

    // Time at constant acceleration
    m_t2 = Math.max(0, (V - m_startVelocity - J * m_t1 * m_t1) / A);

    // Time to reduce acceleration to zero (jerk down phase)
    m_t3 = m_t1;

    // Total acceleration time
    double totalAccelTime = m_t1 + m_t2 + m_t3;

    // Total distance during acceleration
    double accelDistance =
        m_startVelocity * totalAccelTime
            + 0.5 * J * m_t1 * m_t1 * m_t1
            + 0.5 * A * m_t2 * m_t2
            + (J * m_t1 * m_t1 + A * m_t2) * m_t3
            - 0.5 * J * m_t3 * m_t3 * m_t3;

    // Deceleration phases (mirror of acceleration)
    m_t4 = m_t3;
    m_t5 = m_t2;
    m_t6 = m_t1;

    // Total distance during deceleration
    double decelDistance =
        V * (m_t4 + m_t5 + m_t6)
            - 0.5 * J * m_t4 * m_t4 * m_t4
            - 0.5 * A * m_t5 * m_t5
            - (J * m_t4 * m_t4 + A * m_t5) * m_t6
            + 0.5 * J * m_t6 * m_t6 * m_t6;

    // Cruise distance
    double totalDistance = Math.abs(m_targetPosition - m_startPosition);
    m_cruiseDistance = totalDistance - accelDistance - decelDistance;
    double cruiseTime = Math.max(0, m_cruiseDistance / V);

    // Total profile time
    m_totalTime = totalAccelTime + cruiseTime + (m_t4 + m_t5 + m_t6);

    m_currentPhase = ProfilePhase.ACCEL_JERK_UP;
  }

  public State calculate(double deltaT) {
    double remainingTime = deltaT;

    while (remainingTime > 0 && m_currentPhase != ProfilePhase.FINISHED) {
      double phaseTimeLeft = getPhaseRemainingTime();
      double dt = Math.min(remainingTime, phaseTimeLeft);

      switch (m_currentPhase) {
        case ACCEL_JERK_UP:
          updateJerkPhase(dt, m_constraints.maxJerk);
          break;
        case ACCEL_CONSTANT:
          updateConstantAccelPhase(dt);
          break;
        case ACCEL_JERK_DOWN:
          updateJerkPhase(dt, -m_constraints.maxJerk);
          break;
        case CRUISE:
          updateCruisePhase(dt);
          break;
        case DECEL_JERK_DOWN:
          updateJerkPhase(dt, -m_constraints.maxJerk);
          break;
        case DECEL_CONSTANT:
          updateConstantAccelPhase(-dt);
          break;
        case DECEL_JERK_UP:
          updateJerkPhase(dt, m_constraints.maxJerk);
          break;
        default:
          break;
      }

      m_elapsedTime += dt;
      remainingTime -= dt;

      // Check for phase transitions
      if (m_elapsedTime >= getPhaseEndTime(m_currentPhase)) {
        advanceToNextPhase();
      }
    }

    return new State(m_currentPosition, m_currentVelocity);
  }

  private void updateJerkPhase(double dt, double jerk) {
    double newVelocity = m_currentVelocity + jerk * dt * dt / 2;

    double deltaPosition = m_currentVelocity * dt + jerk * dt * dt * dt / 6;

    m_currentPosition += deltaPosition;
    m_currentVelocity = newVelocity;
  }

  private void updateConstantAccelPhase(double dt) {
    double acceleration = m_constraints.maxAcceleration;
    if (m_currentPhase == ProfilePhase.DECEL_CONSTANT) {
      acceleration *= -1;
    }

    m_currentVelocity += acceleration * dt;
    m_currentPosition += m_currentVelocity * dt + 0.5 * acceleration * dt * dt;
  }

  private void updateCruisePhase(double dt) {
    m_currentPosition += m_currentVelocity * dt;
  }

  private double getPhaseEndTime(ProfilePhase phase) {
    switch (phase) {
      case ACCEL_JERK_UP:
        return m_t1;
      case ACCEL_CONSTANT:
        return m_t1 + m_t2;
      case ACCEL_JERK_DOWN:
        return m_t1 + m_t2 + m_t3;
      case CRUISE:
        return m_t1 + m_t2 + m_t3 + (m_cruiseDistance / m_constraints.maxVelocity);
      case DECEL_JERK_DOWN:
        return m_totalTime - m_t6 - m_t5 - m_t4;
      case DECEL_CONSTANT:
        return m_totalTime - m_t6 - m_t5;
      case DECEL_JERK_UP:
        return m_totalTime - m_t6;
      default:
        return m_totalTime;
    }
  }

  private double getPhaseRemainingTime() {
    return getPhaseEndTime(m_currentPhase) - m_elapsedTime;
  }

  private void advanceToNextPhase() {
    switch (m_currentPhase) {
      case ACCEL_JERK_UP:
        m_currentPhase = ProfilePhase.ACCEL_CONSTANT;
        break;
      case ACCEL_CONSTANT:
        m_currentPhase = ProfilePhase.ACCEL_JERK_DOWN;
        break;
      case ACCEL_JERK_DOWN:
        m_currentPhase = m_cruiseDistance > 0 ? ProfilePhase.CRUISE : ProfilePhase.DECEL_JERK_DOWN;
        break;
      case CRUISE:
        m_currentPhase = ProfilePhase.DECEL_JERK_DOWN;
        break;
      case DECEL_JERK_DOWN:
        m_currentPhase = ProfilePhase.DECEL_CONSTANT;
        break;
      case DECEL_CONSTANT:
        m_currentPhase = ProfilePhase.DECEL_JERK_UP;
        break;
      case DECEL_JERK_UP:
        m_currentPhase = ProfilePhase.FINISHED;
        break;
      default:
        break;
    }
  }
}
