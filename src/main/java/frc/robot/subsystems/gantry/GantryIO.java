package frc.robot.subsystems.gantry;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.GantryConstants;
import org.littletonrobotics.junction.AutoLog;

public interface GantryIO extends AutoCloseable {
  @AutoLog
  public class GantryIOInputs {
    public double encoderPosition;
    public double tempCelcius;
    public double appliedVoltage;
    public double currentAmps;
  }

  public default void updateInputs(GantryIOInputs inputs) {}

  public default void setGantryVoltage(double voltage, GantryIOInputs inputs) {}

  public default void setGantryPosition(double pos, GantryIOInputs inputs) {}

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
