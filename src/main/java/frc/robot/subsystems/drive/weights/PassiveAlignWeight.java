package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PassiveAlignWeight implements DriveWeight {
  private DriveWeight inner;

  public PassiveAlignWeight(DriveWeight inner) {
    this.inner = inner;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return inner.getSpeeds();
  }

  @Override
  public double getWeight() {
    
  }
}
