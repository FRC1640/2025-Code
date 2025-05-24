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
  SlewRateLimiter accel = new SlewRateLimiter(3);

  // pose speeds line PIDs
  PIDController linearDrivePid = RobotPIDConstants.constructPID(RobotPIDConstants.linearDrivePID);
  PIDController linearRotatePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  // local align PIDs
  private PIDController localXPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign, "LocalAlignPID_x");
  private PIDController localYPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlign, "LocalAlignPID_y");
  private PIDController localThetaPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.localAnglePid);
  private ProfiledPIDController localDrivePpid =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.localDriveProfiledPid,
          AutoAlignConfig.localAlignPpidConstraints,
          "LocalAlignPPID");
  // private PIDController localYLinearDrivePid =
  //     RobotPIDConstants.constructPID(RobotPIDConstants.localTagAlignY);

  // passive align PIDs
  private PIDController passiveXPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.passiveXPid);
  private PIDController passiveYPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.passiveYPid);
  private PIDController passiveThetaPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.passiveThetaPid);
  private PIDController passiveDrivePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.passiveDrivePid);
  private PIDController passiveRotatePid =
      RobotPIDConstants.constructPID(RobotPIDConstants.passiveRotatePid);

  public AutoAlignHelper() {}

  private static ChassisSpeeds convertToFieldRelative(
      ChassisSpeeds fieldRelative, Gyro gyro, Pose2d robot) {
    Translation2d xy =
        new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    Translation2d rotated =
        xy.rotateBy(
            new Rotation2d(
                    gyro.getOffset() - gyro.getRawAngleRadians() + robot.getRotation().getRadians())
                .unaryMinus());
    // Logger.recordOutput(
    //     "A_DEBUG/speedConversionRotation",
    //     new Rotation2d(
    //             gyro.getOffset() - gyro.getRawAngleRadians() + robot.getRotation().getRadians())
    //         .unaryMinus());
    // Logger.recordOutput("A_DEBUG/gyroOffset", gyro.getOffset());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), fieldRelative.omegaRadiansPerSecond);
  }

  // ---- Linear Align ---- //
  public ChassisSpeeds getPoseSpeedsLine(Pose2d robotPose, Pose2d targetPose, Gyro gyro) {
    Pose2d robot = robotPose;
    Pose2d target = targetPose;
    Rotation2d angleToTarget = robot.getTranslation().minus(target.getTranslation()).getAngle();
    double dist = robot.getTranslation().getDistance(target.getTranslation());
    double linearPID = linearDrivePid.calculate(dist, 0);
    double rotationalPID =
        linearRotatePid.calculate(robot.getRotation().minus(target.getRotation()).getRadians(), 0);
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

  // ---- Local Align ---- //
  public ChassisSpeeds getLocalAlignSpeedsLine(
      Translation2d vector,
      Gyro gyro,
      Rotation2d robotRotation,
      Rotation2d endRotation,
      DriveSubsystem driveSubsystem) {
    localThetaPid.enableContinuousInput(-Math.PI, Math.PI);
    // target origin
    Translation2d target = new Translation2d();
    // take measurements
    double dist = vector.getDistance(target);
    Rotation2d angle = vector.minus(target).getAngle();
    // calculate output

    Logger.recordOutput("localaligndist", dist);
    double vx =
        (dist < 0.14
            ? -localXPid.calculate(vector.getX(), 0)
            : -localDrivePpid.calculate(dist, 0) * angle.getCos());

    double vy =
        (dist < 0.14
            ? -localYPid.calculate(vector.getY(), 0)
            : -localDrivePpid.calculate(dist, 0) * angle.getSin());
    Logger.recordOutput("LocalTagAlign/profiledLocalAlign", dist > 0.14);
    double rotational =
        localThetaPid.calculate(robotRotation.getRadians(), endRotation.getRadians());
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
    localDrivePpid.reset(vector.getNorm(), parallelVel);
  }

  // ---- Passive Align ---- //
  public ChassisSpeeds getPassiveStationSpeedsXY(Pose2d robotPose, Pose2d targetPose, Gyro gyro) {
    // for angle control
    passiveThetaPid.enableContinuousInput(-Math.PI, Math.PI);
    // rotate axes
    Translation2d delta = robotPose.minus(targetPose).getTranslation();
    Translation2d deltaRotated =
        delta.rotateBy(new Rotation2d(delta.getAngle().getRadians() - Math.PI / 4)); // TODO
    // calculate outputs
    double vx = passiveXPid.calculate(deltaRotated.getX(), 0);
    double vy = passiveYPid.calculate(deltaRotated.getY(), 0);
    double vth =
        passiveThetaPid.calculate(
            robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    // clamp within constraints
    vx = MathUtil.clamp(vx, -1, 1);
    vx = MathUtil.applyDeadband(vx, 0.01);
    vx *= DriveConstants.maxSpeed;

    vy = MathUtil.clamp(vy, -1, 1);
    vy = MathUtil.applyDeadband(vy, 0.01);
    vy *= DriveConstants.maxSpeed;

    vth = MathUtil.clamp(vth, -1, 1);
    vth = MathUtil.applyDeadband(vth, 0.01);
    vth *= DriveConstants.maxOmega;
    // convert chassis speeds
    ChassisSpeeds robotRelative = new ChassisSpeeds(vx, vy, vth);
    return convertToFieldRelative(robotRelative, gyro, robotPose);
  }

  public ChassisSpeeds getPassiveStationSpeedsLine(Pose2d robotPose, Pose2d targetPose, Gyro gyro) {
    linearRotatePid.enableContinuousInput(-Math.PI, Math.PI);
    // calculate output
    Rotation2d angleDelta =
        robotPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    double vOutput =
        passiveDrivePid.calculate(
            robotPose.getTranslation().getDistance(targetPose.getTranslation()), 0);
    double omegaOutput =
        passiveRotatePid.calculate(
            robotPose.getRotation().minus(targetPose.getRotation()).getRadians(), 0);
    // clamp output
    vOutput = MathUtil.clamp(vOutput, -1, 1);
    vOutput = MathUtil.applyDeadband(vOutput, 0.01);
    vOutput *= DriveConstants.maxSpeed;

    omegaOutput = MathUtil.clamp(omegaOutput, -1, 1);
    omegaOutput = MathUtil.applyDeadband(omegaOutput, 0.01);
    omegaOutput *= DriveConstants.maxOmega;
    // convert to field-relative
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            vOutput * angleDelta.getCos(), vOutput * angleDelta.getSin(), omegaOutput);
    return convertToFieldRelative(speeds, gyro, robotPose);
  }

  public boolean isStrafing(Rotation2d normal, ChassisSpeeds speeds, double threshold) {
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    boolean strafing = Math.abs(velocity.getAngle().minus(normal).getRadians()) < threshold;
    Logger.recordOutput("PassiveStationAlign/strafing", strafing);
    return strafing;
  }
}
