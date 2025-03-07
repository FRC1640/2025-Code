package frc.robot.util.spark;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

public class SparkPIDConstants {
  public Double kP;
  public Double kI;
  public Double kD;
  public Double minOutput;
  public Double maxOutput;
  public Double velocityFF;
  public Double maxVel;
  public Double maxAccel;
  public Double allowedErr;
  public Double positionConversionFactor;
  public Double velocityConversionFactor;
  public MAXMotionPositionMode maxPositionMode;
  public ClosedLoopSlot closedLoopSlot;
  public String alias;
  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change in error
   * @param closedLoopSlot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.closedLoopSlot = closedLoopSlot;
  }
  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change in error
   * @param positionConversionFactor conversion factor for the Position
   * @param closedLoopSlot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      ClosedLoopSlot closedLoopSlot,
      double positionConversionFactor) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.positionConversionFactor = positionConversionFactor;
    this.closedLoopSlot = closedLoopSlot;
  }
  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change in error
   * @param positionConversionFactor conversion factor for the Position
   * @param velocityConversionFactor conversion factor for the Velocity
   * @param closedLoopSlot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      ClosedLoopSlot closedLoopSlot,
      double positionConversionFactor,
      double velocityConversionFactor) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.positionConversionFactor = positionConversionFactor;
    this.velocityConversionFactor = velocityConversionFactor;
    this.closedLoopSlot = closedLoopSlot;
  }
  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change error
   * @param minOutput Minimum output of PID
   * @param maxOutput Maximum output of PID
   * @param velocityFF Velocity Feed Forward
   * @param maxVel Max Velocity
   * @param maxAccel Max Acceleration
   * @param allowedErr Allowed Error Amount
   * @param maxPositionMode The position mode
   * @param closedLoopSlot The closed loop slot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      double velocityFF,
      double maxVel,
      double maxAccel,
      double allowedErr,
      MAXMotionPositionMode maxPositionMode,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.velocityFF = velocityFF;
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
    this.allowedErr = allowedErr;
    this.closedLoopSlot = closedLoopSlot;
  }
  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change in error
   * @param minOutput Minimum output of PID
   * @param maxOutput Maximum output of PID
   * @param velocityFF Velocity Feed Forward
   * @param closedLoopSlot The closed loop slot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      double velocityFF,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.velocityFF = velocityFF;
    this.closedLoopSlot = closedLoopSlot;
  }

  /**
   * @param kP Proportional Gain
   * @param kI Integral
   * @param kD The rate of change in error
   * @param minOutput Minimum output of PID
   * @param maxOutput Maximum output of PID
   * @param maxVel Max Velocity
   * @param maxAccel Max Acceleration
   * @param closedLoopSlot The closed loop slot
   */
  public SparkPIDConstants(
      double kP,
      double kI,
      double kD,
      double minOutput,
      double maxOutput,
      double maxVel,
      double maxAccel,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
    this.closedLoopSlot = closedLoopSlot;
  }
  /** Set the constraints of output of the PID */
  public SparkPIDConstants setConstraint(double minOutput, double maxOutput) {
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    return this;
  }
  /** Sets the Velocity Feed Forward */
  public SparkPIDConstants setVelocityFF(double velocityFF) {
    this.velocityFF = velocityFF;
    return this;
  }
  /** Sets max velocity */
  public SparkPIDConstants setMaxVelocity(double maxVel) {
    this.maxVel = maxVel;
    return this;
  }
  /** Set Max Acceleration */
  public SparkPIDConstants setMaxAccel(double maxAccel) {
    this.maxAccel = maxAccel;
    return this;
  }
  /** Set Allowed Error */
  public SparkPIDConstants setAllowedErr(double allowedErr) {
    this.allowedErr = allowedErr;
    return this;
  }
  /** Set the MAXPosition mode */
  public SparkPIDConstants setMaxPositionMode(MAXMotionPositionMode maxPositionMode) {
    this.maxPositionMode = maxPositionMode;
    return this;
  }
  /** Set the Closed Loop Slot on the Spark */
  public SparkPIDConstants setClosedLoopSlot(ClosedLoopSlot closedLoopSlot) {
    this.closedLoopSlot = closedLoopSlot;
    return this;
  }
  /** Set Position Conversion Factor */
  public SparkPIDConstants setPositionConversion(Double conversion) {
    this.positionConversionFactor = conversion;
    return this;
  }
  /** Set Velocity Conversion Factor */
  public SparkPIDConstants setVelocityConversion(Double conversion) {
    this.velocityConversionFactor = conversion;
    return this;
  }
  /** Set Alias of the logged SparkPID */
  public SparkPIDConstants setAlias(String alias) {
    this.alias = alias;
    return this;
  }
}
