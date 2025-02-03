package frc.robot.util.tools;

import edu.wpi.first.math.MathUtil;

public class MotorLim {
  /**
   * Modifies the inputted voltage so as to not move out of limits
   *
   * @param pos the current position.
   * @param voltage the base voltage to clamp.
   * @param bounds the limits for the motor
   * @return clamped voltage.
   */
  // public static double applyLimits(
  //     Boolean highLimit, Boolean lowLimit, double pos, double voltage, Bounds bounds) {
  //   double voltageClamped = voltage;
  //   if (!(Double.isNaN(voltageClamped) || Double.isNaN(pos))) {
  //     if (pos < bounds.low.value) {
  //       voltageClamped = Math.max(voltage, 0);
  //     }
  //     if (pos > bounds.high.value) {
  //       voltageClamped = Math.min(voltage, 0);
  //     }
  //   } else {
  //     return 0;
  //   }

  //   return voltageClamped;
  // }

  public static double applyLimits(
      double pos, double voltage, boolean lowLimit, boolean highLimit) {
    double voltageClamped = voltage;
    if (!(Double.isNaN(voltageClamped) || Double.isNaN(pos))) {
      if ((highLimit && voltage > 0) || (lowLimit && voltage < 0)) {
        return 0;
      }
      return voltageClamped;
    } else {
      return 0;
    }
  }

  public static double applyLimits(double pos, double voltage, Double lowLimit, Boolean highLimit) {
    return applyLimits(pos, voltage, pos < lowLimit, highLimit);
  }

  public static double applyLimits(double pos, double voltage, Boolean lowLimit, Double highLimit) {
    return applyLimits(pos, voltage, lowLimit, pos < highLimit);
  }

  public static double applyLimits(double pos, double voltage, Double lowLimit, Double highLimit) {
    return applyLimits(pos, voltage, pos > lowLimit, pos < highLimit);
  }

  /**
   * Clamps voltage of a motor
   *
   * @param voltage the voltage of the motor
   * @return the voltage clamped between -12 and +12
   */
  public static double clampVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    if (Math.abs(voltage) < 0.001) {
      voltage = 0;
    }
    return voltage;
  }
}
