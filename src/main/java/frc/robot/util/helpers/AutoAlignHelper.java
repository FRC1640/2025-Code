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
  SlewRateLimiter cStationAccel = new SlewRateLimiter(3);
  PIDController rotatePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  private PIDController localDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign);
  private PIDController localVelocityPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignVelocity);
  // private PIDController localYLinearDrivePid =
  //     RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignY);
  private PIDController localRotationPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localAnglePid);
  private ProfiledPIDController localDriveProfiledPid =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.localDriveProfiledPid,
          AutoAlignConfig.localAlignPpidConstraints,
          "LocalAlignPPID");

  private PIDController cStationDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.cStationDrivePID);
  // private PIDController cStationYLinearDrivePid =
  //     RobotPIDConstants.constructPID(RobotPIDConstants.cStationTagAlignY);
  private PIDController cStationRotationPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.cStationAnglePid);
  private ProfiledPIDController cStationDriveProfiledPid =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.cStationDrivePPID,
          AutoAlignConfig.cStationPpidConstraints,
          "cStationAlignPPID");

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
    double ySpeed = -Math.sin(angleToTarget.getRadians()) * linearPID;
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
    Logger.recordOutput(
        "A_DEBUG/speedConversionRotation",
        new Rotation2d(
                gyro.getOffset() - gyro.getRawAngleRadians() + robot.getRotation().getRadians())
            .unaryMinus());
    Logger.recordOutput("A_DEBUG/gyroOffset", gyro.getOffset());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), fieldRelative.omegaRadiansPerSecond);
  }

  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector,
      Gyro gyro,
      Rotation2d robotRotation,
      Rotation2d endRotation,
      DriveSubsystem driveSubsystem) {
    localRotationPid.enableContinuousInput(-Math.PI, Math.PI);
    // target origin
    Translation2d target = new Translation2d();
    // take measurements
    double dist = vector.getDistance(target);
    Rotation2d angle = vector.minus(target).getAngle();
    // calculate output

    Logger.recordOutput("localaligndist", dist);
    double linear =
        dist < 0.05 ? localDrivePid.calculate(dist, 0) : localDriveProfiledPid.calculate(dist, 0);
    Logger.recordOutput("LocalTagAlign/profiledLocalAlign", dist > 0.05);
    double rotational =
        localRotationPid.calculate(robotRotation.getRadians(), endRotation.getRadians());
    // convert to percentage
    linear = MathUtil.clamp(linear, -1, 1);
    linear = MathUtil.applyDeadband(linear, 0.01);
    linear *= DriveConstants.maxSpeed; // TODO scale max speed for acceleration?

    rotational = MathUtil.clamp(rotational, -1, 1);
    rotational = MathUtil.applyDeadband(rotational, 0.01);
    rotational *= DriveConstants.maxOmega;
    // limit rate
    linear = accel.calculate(linear);
    // find component vectors
    double vx = -linear * angle.getCos();
    double vy = -linear * angle.getSin();
    // convert chassis speeds
    ChassisSpeeds robotRelative = new ChassisSpeeds(vx, vy, rotational);
    return convertToFieldRelative(robotRelative, gyro, new Pose2d());
  }

  public void resetLocalMotionProfile(Translation2d vector, DriveSubsystem driveSubsystem) {
    ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
    // System.out.println(speeds); // goofy stuff
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double parallelVel =
        -driveSubsystem.chassisSpeedsMagnitude()
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

  public ChassisSpeeds getCStationAlignSpeedsLine(
      Pose2d robotPose, Pose2d targetPose, Gyro gyro, DriveSubsystem driveSubsystem) {
    Pose2d robot = robotPose;
    Pose2d target = targetPose;
    // take measurements
    double dist = robot.getTranslation().getDistance(target.getTranslation());
    Rotation2d angleToTarget = robot.getTranslation().minus(target.getTranslation()).getAngle();
    // calculate output

    Logger.recordOutput("cStationaligndist", dist);
    double linear =
        dist < 0.05
            ? cStationDrivePid.calculate(dist, 0)
            : cStationDriveProfiledPid.calculate(dist, 0);
    Logger.recordOutput("cStationAlign/profiledcStationAlign", dist > 0.05);
    double rotational =
        cStationRotationPid.calculate(
            robot.getRotation().minus(target.getRotation()).getRadians(), 0);
    // convert to percentage
    linear = MathUtil.clamp(linear, -1, 1);
    linear = MathUtil.applyDeadband(linear, 0.01);
    linear *= DriveConstants.maxSpeed; // TODO scale max speed for acceleration?

    rotational = MathUtil.clamp(rotational, -1, 1);
    rotational = MathUtil.applyDeadband(rotational, 0.01);
    rotational *= DriveConstants.maxOmega;
    // limit rate
    linear = cStationAccel.calculate(linear);
    // find component vectors
    double vx = -linear * angleToTarget.getCos();
    double vy = -linear * angleToTarget.getSin();
    // convert chassis speeds
    ChassisSpeeds robotRelative = new ChassisSpeeds(vx, vy, rotational);
    return convertToFieldRelative(robotRelative, gyro, new Pose2d());
  }

  public void resetCStationMotionProfile(Pose2d robot, DriveSubsystem driveSubsystem) {
    ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
    // System.out.println(speeds); // goofy stuff
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double parallelVel =
        -driveSubsystem.chassisSpeedsMagnitude()
            * (velocity.getAngle().minus(robot.getRotation())).getCos();
    cStationDriveProfiledPid.reset(robot.getRotation().getRadians(), parallelVel);
  }
}
