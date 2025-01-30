package frc.robot.subsystems.intakeoutake;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.GantryConstants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeOutakeIO extends AutoCloseable {

  @AutoLog
  public class IntakeOutakeIOInputs {
    public double tempCelcius;
    public double appliedVoltage;
  }

  public default void setIntakeVoltage(double voltage) {}

  public default void updateInputs(IntakeOutakeIOInputs inputs) {}

  /**
   * Modifies the inputted voltage so as to not move out of limits
   *
   * @param pos the current position.
   * @param voltage the base voltage to clamp.
   * @return clamped voltage.
   */
  public default double applyLimits(double pos, double voltage) {
    double voltageClamped = voltage;
    if (!(Double.isNaN(voltageClamped) || Double.isNaN(pos))) {
      if (pos < GantryConstants.leftLimit) {
        voltageClamped = Math.max(voltage, 0);
      }
      if (pos > GantryConstants.rightLimit) {
        voltageClamped = Math.min(voltage, 0);
      }
    } else {
      return 0;
    }

    return voltageClamped;
  }

  public default double clampVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    if (Math.abs(voltage) < 0.001) {
      voltage = 0;
    }
    return voltage;
  }

  @Override
  default void close() {}
}
