package frc.robot.util.helpers;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.misc.AllianceManager;
import java.util.ArrayList;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class AutoAlignHelper {
  PIDController linearDrivePID = RobotPIDConstants.constructPID(RobotPIDConstants.linearDrivePID);
  SlewRateLimiter accel = new SlewRateLimiter(3);
  PIDController rotatePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  private PIDController localXLinearDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignX);
  private PIDController localYLinearDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignY);
  private PIDController localRotationPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localAnglePid);

  public ChassisSpeeds getPoseSpeedsLine(Pose2d robotPose, Pose2d targetPose, Gyro gyro) {
    Pose2d robot = robotPose;
    Pose2d target = targetPose;
    Rotation2d angleToTarget = robot.getTranslation().minus(target.getTranslation()).getAngle();
    double dist = robot.getTranslation().getDistance(target.getTranslation());
    double linearPID = linearDrivePID.calculate(dist, 0);
    double rotationalPID =
        rotatePID.calculate(robot.getRotation().minus(target.getRotation()).getRadians(), 0);
    linearPID = MathUtil.clamp(linearPID, -1, 1);
    rotationalPID = MathUtil.clamp(rotationalPID, -1, 1);
    linearPID = MathUtil.applyDeadband(linearPID, 0.01);
    rotationalPID = MathUtil.applyDeadband(rotationalPID, 0.01);
    linearPID *= DriveConstants.maxSpeed;
    rotationalPID *= DriveConstants.maxOmega;
    linearPID = accel.calculate(linearPID);

    double xSpeed = Math.cos(angleToTarget.getRadians()) * linearPID;
    double ySpeed = Math.sin(angleToTarget.getRadians()) * linearPID;
    Logger.recordOutput("Drive/AutoAlignPosition", target);

    // convert to robot relative from field relative
    ChassisSpeeds fieldRelative = new ChassisSpeeds(xSpeed, ySpeed, rotationalPID);
    return convertToFieldRelative(fieldRelative, gyro, robot);
  }

  public static ChassisSpeeds convertToFieldRelative(
      ChassisSpeeds fieldRelative, Gyro gyro, Pose2d robot) {
    Translation2d xy =
        new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    Translation2d rotated =
        xy.rotateBy(
            new Rotation2d(
                    gyro.getOffset() - gyro.getRawAngleRadians() + robot.getRotation().getRadians())
                .unaryMinus());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), fieldRelative.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds convertToFieldRelative(
      ChassisSpeeds fieldRelative, Rotation2d robotRotation) {
    Translation2d xy =
        new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    Logger.recordOutput("A_DEBUG/speedsNotRotated", new Pose2d(xy, new Rotation2d()));
    Translation2d rotated = xy.rotateBy(robotRotation);
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), fieldRelative.omegaRadiansPerSecond);
  }

  /*
  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector, Rotation2d robotRotation, Rotation2d endRotation) {
    // measure error
    Rotation2d deltaTheta = endRotation.minus(robotRotation);
    double dist = vector.getNorm();
    // calculate percentages
    double linearOutput = localXLinearDrivePid.calculate(dist, 0);
    double rotationalOutput = localRotationPid.calculate(deltaTheta.getRadians(), 0);
    // convert output to velocity
    linearOutput = MathUtil.clamp(linearOutput, -1, 1);
    linearOutput = MathUtil.applyDeadband(linearOutput, 0.01);
    linearOutput *= DriveConstants.maxSpeed;

    rotationalOutput = MathUtil.clamp(rotationalOutput, -1, 1);
    rotationalOutput = MathUtil.applyDeadband(rotationalOutput, 0.01);
    rotationalOutput *= DriveConstants.maxOmega;
    // filter slew rate
    linearOutput = accel.calculate(linearOutput);

    // convert to robot-centric speeds
    double vectorAngle = Math.atan2(vector.getY(), vector.getX());
    double xSpeed = Math.cos(deltaTheta.getRadians()) * linearOutput;
    double ySpeed = Math.sin(deltaTheta.getRadians()) * linearOutput;
    Logger.recordOutput("Drive/LocalAlignPosition", vector);

    // convert to robot relative from field relative
    ChassisSpeeds fieldRelative = new ChassisSpeeds(xSpeed, ySpeed, rotationalOutput);
    // return convertToFieldRelative(fieldRelative, robotRotation.unaryMinus());
    return fieldRelative;
  }
  */

  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector, Rotation2d robotRotation, Rotation2d endRotation) {
    double x = -localXLinearDrivePid.calculate(vector.getX(), 0);
    double y = -localYLinearDrivePid.calculate(vector.getY(), 0);
    double rot = localRotationPid.calculate(robotRotation.getRadians(), endRotation.getRadians());
    return convertToFieldRelative(new ChassisSpeeds(x, y, rot), robotRotation);
  }

  public static AprilTag getAutoalignTagId(Pose2d target) {
    ArrayList<AprilTag> autoalignTags = new ArrayList<>();
    IntStream.of(
            AllianceManager.chooseFromAlliance(
                new int[] {17, 18, 19, 20, 21, 22}, new int[] {6, 7, 8, 9, 10, 11}))
        .forEach(
            (i) ->
                autoalignTags.add(
                    new AprilTag(i, FieldConstants.aprilTagLayout.getTagPose(i).get())));
    AprilTag nearestTag = autoalignTags.get(0);
    double nearestDist = Double.MAX_VALUE;
    for (AprilTag tag : autoalignTags) {
      double dist =
          target.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d());
      if (dist < nearestDist) {
        nearestTag = tag;
        nearestDist = dist;
      }
    }
    return nearestTag;
  }
}
