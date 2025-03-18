package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public class AlgaeIOInputs {
    public double intakeMotorLeftVelocity = 0.0;
    public double intakeMotorLeftCurrent = 0.0;
    public double intakeMotorLeftVoltage = 0.0;
    public double intakeMotorLeftTemperature = 0.0;

    public double intakeMotorRightVelocity = 0.0;
    public double intakeMotorRightCurrent = 0.0;
    public double intakeMotorRightVoltage = 0.0;
    public double intakeMotorRightTemperature = 0.0;

    public boolean solenoidForward = false;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double left, double right) {}

  public default void setSolenoid(boolean set) {}

  public default boolean hasSimAlgae() {
    return false;
  }

  public default void updateEMA(double data) {}
}
