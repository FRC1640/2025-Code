package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.gyro.Gyro;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathplannerWeight implements DriveWeight {
  private static ChassisSpeeds speeds = new ChassisSpeeds();
  private static Gyro gyro;
  private static Supplier<Pose2d> robotPose;
  public static Pose2d setpoint = new Pose2d(0.00001, 0.00001, new Rotation2d(0.00001));
  private static Supplier<Pose2d> getTarget;

  public PathplannerWeight(Gyro gyro, Supplier<Pose2d> robotPose, Supplier<Pose2d> target) {
    PathplannerWeight.gyro = gyro;
    PathplannerWeight.robotPose = robotPose;
    PathplannerWeight.getTarget = target;
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

  public static boolean seesTarget() {
    Pose2d robot = robotPose.get();
    DoubleFunction<Double> sightline =
        (x) ->
            Math.tan(robot.getRotation().getRadians()) * (x - robot.getTranslation().getX())
                + robot.getTranslation().getY();
    Pose2d target = getTarget.get();
    // Logger.recordOutput("Drive/FollowPathNearest/odometry", robot);
    boolean sees = (sightline.apply(target.getX()) - target.getY()) < 0.6; // TODO
    Logger.recordOutput("Drive/FollowPathNearest/odometry_conditions", sees && nearEnd());
    return sees;
  }

  public static boolean nearEnd() {
    double distance = Math.abs(robotPose.get().minus(getTarget.get()).getTranslation().getNorm());
    return distance < AutoAlignConfig.trigDistThreshold && distance > 0.1;
  }
}
