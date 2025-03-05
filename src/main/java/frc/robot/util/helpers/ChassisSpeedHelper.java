package frc.robot.util.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedHelper {
  public static Rotation2d angleOf(ChassisSpeeds speeds) {
    if (magnitude(speeds) < 0.001) {
      return new Rotation2d();
    }

    return new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static double magnitude(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }
}
