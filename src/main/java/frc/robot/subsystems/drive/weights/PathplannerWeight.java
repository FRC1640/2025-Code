package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathplannerWeight implements DriveWeight {
  private static ChassisSpeeds speeds = new ChassisSpeeds();
  private Gyro gyro;
  private Supplier<Pose2d> robotPose;

  public PathplannerWeight(Gyro gyro, Supplier<Pose2d> robotPose) {
    this.gyro = gyro;
    this.robotPose = robotPose;
  }

  public static void setSpeeds(ChassisSpeeds newSpeeds) {
    Logger.recordOutput("Pathplanner/InputSpeeds", newSpeeds);
    speeds = newSpeeds;
    Logger.recordOutput("Pathplanner/SetSpeeds", speeds);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Logger.recordOutput("Pathplanner/SetSpeeds", speeds);
    Translation2d speedsNotRotated =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    speedsNotRotated = speedsNotRotated.rotateBy(gyro.getAngleRotation2d());

    ChassisSpeeds speedsRotated =
        new ChassisSpeeds(
            speedsNotRotated.getX(), speedsNotRotated.getY(), speeds.omegaRadiansPerSecond);
    return speedsRotated;
  }
}
