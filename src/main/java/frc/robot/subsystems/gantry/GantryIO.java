package frc.robot.subsystems.gantry;

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

  public default void setGantrySpeedVoltage(double voltage) {}

  public default void setGantrySpeedPercent(double speed) {}

  public default void setCarriagePosition(double pos, GantryIOInputs inputs) {}

  /**
   * Modifies the inputted speed so as to not move out of limits
   *
   * @param pos the current position.
   * @param speed the base speed to clamp.
   * @return clamped speed.
   */
  public default double clampSpeeds(double pos, double speed) {
    double speedClamped = speed;
    if (!(Double.isNaN(speedClamped) || Double.isNaN(pos))) {
      if (pos < GantryConstants.leftLimit) {
        speedClamped = Math.max(speed, 0);
      }
      if (pos > GantryConstants.rightLimit) {
        speedClamped = Math.min(speed, 0);
      }
    } else {
      return 0;
    }

    return speedClamped;
  }

  @Override
  default void close() {}
}
