package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;

public class AutoAlignHelper {
  PIDController linearDrivePID = RobotPIDConstants.constructPID(RobotPIDConstants.linearDrivePID);
  PIDController rotatePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  public Pose2d findNearest(Pose2d[] points, Pose2d robotPose) {
    if (points.length == 0) {
      return null;
    }
    Pose2d nearest = points[0];
    double nearestDist = robotPose.getTranslation().getDistance(nearest.getTranslation());
    for (Pose2d point : points) {
      double dist = robotPose.getTranslation().getDistance(point.getTranslation());
      if (dist < nearestDist) {
        nearest = point;
        nearestDist = dist;
      }
    }
    return nearest;
  }

  public ChassisSpeeds getPoseSpeeds(Pose2d robotPose, Pose2d targetPose, Gyro gyro) {
    Pose2d robot = robotPose;
    Pose2d target = targetPose;
    Rotation2d angleToTarget = robot.getTranslation().minus(target.getTranslation()).getAngle();
    double dist = robot.getTranslation().getDistance(target.getTranslation());
    double linearPID = linearDrivePID.calculate(dist, 0);
    double rotationalPID =
        rotatePID.calculate(robot.getRotation().getRadians(), target.getRotation().getRadians());
    linearPID = MathUtil.clamp(linearPID, -1, 1);
    rotationalPID = MathUtil.clamp(rotationalPID, -1, 1);
    linearPID = MathUtil.applyDeadband(linearPID, 0.01);
    rotationalPID = MathUtil.applyDeadband(rotationalPID, 0.01);
    linearPID *= DriveConstants.maxSpeed;
    rotationalPID *= DriveConstants.maxOmega;

    double xSpeed = Math.cos(angleToTarget.getRadians()) * linearPID;
    double ySpeed = Math.sin(angleToTarget.getRadians()) * linearPID;

    // convert to robot relative from field relative
    Translation2d fieldRelative = new Translation2d(xSpeed, ySpeed);
    fieldRelative =
        fieldRelative.rotateBy(
            new Rotation2d(
                    gyro.getOffset() - gyro.getRawAngleRadians() + robot.getRotation().getRadians())
                .unaryMinus());
    return new ChassisSpeeds(fieldRelative.getX(), fieldRelative.getY(), rotationalPID);
  }
}
