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
  public MAXMotionPositionMode maxPositionMode;
  public ClosedLoopSlot closedLoopSlot;

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
  /*
   * Set the constraints of output of the PID
   */
  public SparkPIDConstants setConstraint(double minOutput, double maxOutput) {
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    return this;
  }
  /*
   * Sets the Velocity Feed Forward
   */
  public SparkPIDConstants setVelocityFF(double velocityFF) {
    this.velocityFF = velocityFF;
    return this;
  }
  /*
   * Sets max velocity
   */
  public SparkPIDConstants setMaxVelocity(double maxVel) {
    this.maxVel = maxVel;
    return this;
  }
  /*
   * Set Max Acceleration
   *
   */
  public SparkPIDConstants setMaxAccel(double maxAccel) {
    this.maxAccel = maxAccel;
    return this;
  }
  /*
   * Set Allowed Error
   */
  public SparkPIDConstants setAllowedErr(double allowedErr) {
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
