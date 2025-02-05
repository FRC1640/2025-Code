package frc.robot.util.spark;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

public class SparkPIDConstants {
  public Double kP = null;
  public Double kI = null;
  public Double kD = null;
  public Double minOutput = null;
  public Double maxOutput = null;
  public Double velocityFF = null;
  public Double maxVel = null;
  public Double maxAccel = null;
  public Double allowedErr = null;
  public MAXMotionPositionMode maxPositionMode = null;
  public ClosedLoopSlot closedLoopSlot = null;

  /**
   * @param kP Proportional Gain
   * @param kI Derivative
   * @param kD The rate of change in error
   * @param closedLoopSlot
   */
  public SparkPIDConstants(double kP, double kI, double kD, ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.closedLoopSlot = closedLoopSlot;
  }
  /**
   * @param kP Proportional Gain
   * @param kI Derivative
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
  /*
   * Set the constraints of output of the PID
   */
  public SparkPIDConstants setConstraint(Double minOutput, Double maxOutput) {
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    return this;
  }
  /*
   * Sets the Velocity Feed Forward
   */
  public SparkPIDConstants setVelocityFF(Double velocityFF) {
    this.velocityFF = velocityFF;
    return this;
  }
  /*
   * Sets max velocity
   */
  public SparkPIDConstants setMaxVelocity(Double maxVel) {
    this.maxVel = maxVel;
    return this;
  }
  /*
   * Set Max Acceleration
   *
   */
  public SparkPIDConstants setMaxAccel(Double maxAccel) {
    this.maxAccel = maxAccel;
    return this;
  }
  /*
   * Set Allowed Error
   */
  public SparkPIDConstants setAllowedErr(Double allowedErr) {
    this.allowedErr = allowedErr;
    return this;
  }
  /*
   * Set the MAXPosition mode
   */
  public SparkPIDConstants setMaxPositionMode(MAXMotionPositionMode maxPositionMode) {
    this.maxPositionMode = maxPositionMode;
    return this;
  }
  /*
   * Set the Closed Loop Slot on the Spark
   */
  public SparkPIDConstants setClosedLoopSlot(ClosedLoopSlot closedLoopSlot) {
    this.closedLoopSlot = closedLoopSlot;
    return this;
  }
}
