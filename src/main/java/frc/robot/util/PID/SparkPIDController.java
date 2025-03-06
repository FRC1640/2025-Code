package frc.robot.util.PID;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import org.littletonrobotics.junction.Logger;

public class SparkPIDController {
  public SparkClosedLoopController sparkClosedLoopController;
  public double set;
  public ControlType controlType;
  public String alias;

  public SparkPIDController(SparkClosedLoopController sparkClosedLoopController, String alias) {
    this.sparkClosedLoopController = sparkClosedLoopController;
    this.alias = alias;
  }

  /**
   * Set the controller reference value based on the selected control mode.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the RelativeEncoder class
   * @param ctrl the control type
   */
  public void setReference(double value, SparkBase.ControlType ctrl) {
    sparkClosedLoopController.setReference(value, ctrl);
  }

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the RelativeEncoder class
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   */
  public void setReference(double value, SparkBase.ControlType ctrl, ClosedLoopSlot slot) {
    Logger.recordOutput("REVClosedLoop/" + alias + "/set reference value", value);
    Logger.recordOutput("REVClosedLoop/" + alias + "/ControlType", ctrl.toString());
    Logger.recordOutput("REVClosedLoop/" + alias + "/ClosedLoopSlot", slot.toString());
    sparkClosedLoopController.setReference(value, ctrl, slot);
  }

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the RelativeEncoder class
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   */
  public void setReference(
      double value, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward) {
    Logger.recordOutput("REVClosedLoop/" + alias + "/set reference value", value);
    Logger.recordOutput("REVClosedLoop/" + alias + "/ControlType", ctrl.toString());
    Logger.recordOutput("REVClosedLoop/" + alias + "/ClosedLoopSlot", slot.toString());
    Logger.recordOutput("REVClosedLoop/" + alias + "/ArbFeedForward", arbFeedforward);
    sparkClosedLoopController.setReference(value, ctrl, slot, arbFeedforward);
  }

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the RelativeEncoder class
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @param arbFFUnits The units the arbitrary feed forward term is in
   */
  public void setReference(
      double value,
      SparkBase.ControlType ctrl,
      ClosedLoopSlot slot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits) {
    Logger.recordOutput("REVClosedLoop/" + alias + "/set reference value", value);
    Logger.recordOutput("REVClosedLoop/" + alias + "/ControlType", ctrl.toString());
    Logger.recordOutput("REVClosedLoop/" + alias + "/ClosedLoopSlot", slot.toString());
    Logger.recordOutput("REVClosedLoop/" + alias + "/ArbFeedForward", arbFeedforward);
    Logger.recordOutput("REVClosedLoop/" + alias + "/arbFFUnits", arbFFUnits.toString());

    sparkClosedLoopController.setReference(value, ctrl, slot, arbFeedforward, arbFFUnits);
  }

  /**
   * Set the I accumulator of the closed loop controller. This is useful when wishing to force a
   * reset on the I accumulator of the closed loop controller. You can also preset values to see how
   * it will respond to certain I characteristics
   *
   * <p>To use this function, the controller must be in a closed loop control mode by calling
   * setReference()
   *
   * @param iAccum The value to set the I accumulator to
   */
  public void setIAccum(double iAccum) {
    Logger.recordOutput("REVClosedLoop/" + alias + "/Set IAccum", iAccum);
    sparkClosedLoopController.setIAccum(iAccum);
  }
}
