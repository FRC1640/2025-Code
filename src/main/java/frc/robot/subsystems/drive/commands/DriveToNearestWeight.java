package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.weights.DriveWeight;

public class DriveToNearestWeight implements DriveWeight {
  @Override
  public ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds();
  }
}
