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

  private PIDController localDrivePid_x =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign, "LocalAlignPID_x");

  private PIDController localDrivePid_y =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign, "LocalAlignPID_y");
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
    double vx =
        (dist < 0.14
            ? -localDrivePid_x.calculate(vector.getX(), 0)
            : -localDriveProfiledPid.calculate(dist, 0) * angle.getCos());

    double vy =
        (dist < 0.14
            ? -localDrivePid_y.calculate(vector.getY(), 0)
            : -localDriveProfiledPid.calculate(dist, 0) * angle.getSin());
    Logger.recordOutput("LocalTagAlign/profiledLocalAlign", dist > 0.14);
    double rotational =
        localRotationPid.calculate(robotRotation.getRadians(), endRotation.getRadians());
    // convert to percentage
    vx = MathUtil.clamp(vx, -1, 1);
    vx = MathUtil.applyDeadband(vx, 0.01);
    vx *= DriveConstants.maxSpeed;

    vy = MathUtil.clamp(vy, -1, 1);
    vy = MathUtil.applyDeadband(vy, 0.01);
    vy *= DriveConstants.maxSpeed;

    rotational = MathUtil.clamp(rotational, -1, 1);
    rotational = MathUtil.applyDeadband(rotational, 0.01);
    rotational *= DriveConstants.maxOmega;
    // limit rate
    // vx = accel.calculate(vx);
    // vy = accel.calculate(vy);
    Logger.recordOutput("actualvector", vector);
    // convert chassis speeds
    ChassisSpeeds robotRelative = new ChassisSpeeds(vx, vy, rotational);
    return convertToFieldRelative(robotRelative, gyro, new Pose2d());
  }

  public void resetLocalMotionProfile(Translation2d vector, DriveSubsystem driveSubsystem) {
    ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
    System.out.println(speeds);
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
}
