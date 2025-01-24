package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public class LiftIOInputs {
    public int rawEncoderValue;
    public double encoderVoltage;
    public double encoderRelative;
    public boolean liftConnected = false;
    public double liftPositionMeters;
    public double liftVelocityMetersPerSecond;
    public double liftAppliedVoltage;
    public double liftCurrentAmps;

    public double tempCelcius;
    public double appliedVoltage;
    public double currentAmps;

    public double[] liftVelocities = new double[] {};
  }

  public default void updateInputs(LiftIOInputs inputs) {}

  public default void setLiftVoltage(double voltage) {}

  public default void setLiftPosition(int pos) {} // different set positions

  // public default void setLiftVelocity(double velocity, LiftIOInputs inputs) {} not sure if needed

  @Override
  default void close() {}
}
