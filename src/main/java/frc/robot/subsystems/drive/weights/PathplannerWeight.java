package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathplannerWeight implements DriveWeight {
  private static ChassisSpeeds speeds = new ChassisSpeeds();
  private static Gyro gyro;
  private static Supplier<Pose2d> robotPose;
  public static Pose2d setpoint = new Pose2d(0.00001, 0.00001, new Rotation2d(0.00001));

  public PathplannerWeight(Gyro gyro, Supplier<Pose2d> robotPose) {
    PathplannerWeight.gyro = gyro;
    PathplannerWeight.robotPose = robotPose;
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

  public static ChassisSpeeds getSpeedsPath() {
    return speeds;
  }

  public static void overrideRotation(DoubleSupplier rotFeedback) {
    PPHolonomicDriveController.overrideRotationFeedback(rotFeedback);
  }

  public static void clearRotationOverride() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  public static void setPoseSupplier(Supplier<Pose2d> supplier) {
    robotPose = supplier;
  }

  public static Pose2d getRobotPose() {
    return robotPose.get();
  }
}
