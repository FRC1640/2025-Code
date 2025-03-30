package frc.robot.util.helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutoAlignHelper {
  PIDController linearDrivePID = RobotPIDConstants.constructPID(RobotPIDConstants.linearDrivePID);
  SlewRateLimiter accel = new SlewRateLimiter(3);
  PIDController rotatePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  private PIDController localDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign);
  // private PIDController localYLinearDrivePid =
  //     RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignY);
  private PIDController localRotationPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localAnglePid);
  private ProfiledPIDController localDriveProfiledPid =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.localDriveProfiledPid,
          AutoAlignConfig.localAlignPpidConstraints,
          "LocalAlignPPID");

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

  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector, Rotation2d robotRotation, Rotation2d endRotation) {
    // target origin
    Translation2d target = new Translation2d();
    // measurements
    Rotation2d angleToTarget = vector.minus(target).getAngle();
    double dist = vector.getDistance(target);
    // calculate percent output
    double linearPid =
        dist < AutoAlignConfig.profiledDistThreshold
            ? localDrivePid.calculate(dist, 0)
            : localDriveProfiledPid.calculate(dist, 0); // TODO velocity control?
    double rotationalPid =
        localRotationPid.calculate(robotRotation.minus(endRotation).getRadians(), 0);
    // convert to velocities
    linearPid = MathUtil.clamp(linearPid, -1, 1);
    linearPid = MathUtil.applyDeadband(linearPid, 0.01);
    linearPid *= DriveConstants.maxSpeed * 0.8;

    rotationalPid = MathUtil.clamp(rotationalPid, -1, 1);
    rotationalPid = MathUtil.applyDeadband(rotationalPid, 0.01);
    rotationalPid *= DriveConstants.maxOmega;
    // limit slew rate
    linearPid = accel.calculate(linearPid);
    // convert to xy components
    double xSpeed = -Math.cos(angleToTarget.getRadians()) * linearPid;
    double ySpeed = -Math.sin(angleToTarget.getRadians()) * linearPid;
    Logger.recordOutput("Drive/LocalAlignVector", vector);

    Logger.recordOutput("A_DEBUG/AngleToTarget", angleToTarget);
    Logger.recordOutput("A_DEBUG/distance", dist);
    Logger.recordOutput("A_DEBUG/linearPid", linearPid);
    Logger.recordOutput("A_DEBUG/rotationalPid", rotationalPid);
    Logger.recordOutput("A_DEBUG/xSpeed", xSpeed);
    Logger.recordOutput("A_DEBUG/ySpeed", ySpeed);
    // convert robot relative to field relative
    ChassisSpeeds robotRelative = new ChassisSpeeds(xSpeed, ySpeed, rotationalPid);
    return convertToFieldRelative(robotRelative, robotRotation);
  }

  public void resetLocalMotionProfile(Translation2d vector, DriveSubsystem driveSubsystem) {
    ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double parallelVel =
        driveSubsystem.chassisSpeedsMagnitude()
            * (velocity.getAngle().minus(vector.getAngle())).getCos();
    localDriveProfiledPid.reset(vector.getNorm(), parallelVel);
  }

  /*
  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector, Rotation2d robotRotation, Rotation2d endRotation) {
    // calculate percentages
    double x = -localXLinearDrivePid.calculate(vector.getX(), 0);
    double y = -localYLinearDrivePid.calculate(vector.getY(), 0);
    double rot =
        localRotationPid.calculate(
            robotRotation.getRadians(),
            AprilTagAlignHelper.clampToInterval(endRotation, robotRotation).getRadians());
    // convert to velocity
    x = MathUtil.applyDeadband(MathUtil.clamp(x, -1, 1), 0.01);
    y = MathUtil.applyDeadband(MathUtil.clamp(y, -1, 1), 0.01);
    double angle = MathUtil.clamp(Math.abs(Math.atan2(y, x)), 0, Math.PI);
    x *= Math.cos(angle) * DriveConstants.maxSpeed * 0.3;
    y *= Math.sin(angle) * DriveConstants.maxSpeed * 0.3;

    rot = MathUtil.applyDeadband(MathUtil.clamp(rot, -1, 1), 0.01) * DriveConstants.maxOmega;
    // convert to field-centric
    return convertToFieldRelative(new ChassisSpeeds(x, y, rot), robotRotation);
  } */
}
