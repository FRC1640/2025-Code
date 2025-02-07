package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathplannerWeight implements DriveWeight {
  private static ChassisSpeeds speeds = new ChassisSpeeds();

  public static void setSpeeds(ChassisSpeeds newSpeeds) {
    speeds = newSpeeds;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return speeds;
  }
}
