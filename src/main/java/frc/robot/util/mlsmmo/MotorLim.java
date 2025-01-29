package frc.robot.util.mlsmmo;

import edu.wpi.first.math.MathUtil;

public class MotorLim {
  /**
   * Modifies the inputted voltage so as to not move out of limits
   *
   * @param pos the current position.
   * @param voltage the base voltage to clamp.
   * @return clamped voltage.
   */
  public static double applyLimits(double pos, double voltage, MotorLimit ideaParm) {
    double voltageClamped = voltage;
    if (!(Double.isNaN(voltageClamped) || Double.isNaN(pos))) {
      if (pos < ideaParm.low) {
        voltageClamped = Math.max(voltage, 0);
      }
      if (pos > ideaParm.high) {
        voltageClamped = Math.min(voltage, 0);
      }
    } else {
      return 0;
    }

    return voltageClamped;
  }

  /*
   * Clamps voltage of a motor
   *
   * @param voltage the voltage of the motor
   * @return the voltage clamped between 1-12
   */
  public static double clampVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    if (Math.abs(voltage) < 0.001) {
      voltage = 0;
    }
    return voltage;
  }
}
