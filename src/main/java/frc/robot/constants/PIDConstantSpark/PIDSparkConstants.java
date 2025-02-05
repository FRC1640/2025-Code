package frc.robot.constants.PIDConstantSpark;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

public class PIDSparkConstants {
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

  public PIDSparkConstants(
      Double kP,
      Double kI,
      Double kD,
      Double minOutput,
      Double maxOutput,
      Double velocityFF,
      Double maxVel,
      Double maxAccel,
      Double allowedErr,
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
  }

  public PIDSparkConstants(Double kP, Double kI, Double kD, ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.closedLoopSlot = closedLoopSlot;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }

  public PIDSparkConstants(
      Double kP,
      Double kI,
      Double kD,
      Double minOutput,
      Double maxOutput,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.closedLoopSlot = closedLoopSlot;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }

  public PIDSparkConstants(
      Double kP, Double kI, Double kD, Double velocityFF, ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.closedLoopSlot = closedLoopSlot;
    this.velocityFF = velocityFF;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }

  public PIDSparkConstants(
      Double kP,
      Double kI,
      Double kD,
      Double minOutput,
      Double maxOutput,
      Double velocityFF,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.velocityFF = velocityFF;
    this.closedLoopSlot = closedLoopSlot;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }

  public PIDSparkConstants(
      Double kP,
      Double kI,
      Double kD,
      Double minOutput,
      Double maxOutput,
      Double velocityFF,
      Double maxVel,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.velocityFF = velocityFF;
    this.closedLoopSlot = closedLoopSlot;
    this.maxVel = maxVel;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }

  public PIDSparkConstants(
      Double kP,
      Double kI,
      Double kD,
      Double minOutput,
      Double maxOutput,
      Double velocityFF,
      Double maxVel,
      Double maxAccel,
      ClosedLoopSlot closedLoopSlot) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
    this.velocityFF = velocityFF;
    this.closedLoopSlot = closedLoopSlot;
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
    maxPositionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
  }
}
