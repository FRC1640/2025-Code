package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.tools.AutoAlignHelper;
import frc.robot.util.tools.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class DriveToNearestWeight implements DriveWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d[]> targetPoses;
  private Gyro gyro;
  private Function<Pose2d, Pose2d> poseFunction;
  AutoAlignHelper autoAlignHelper = new AutoAlignHelper();
  private boolean enabled = false;
  private DriveSubsystem driveSubsystem;

  public DriveToNearestWeight(
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d[]> targetPoses,
      Gyro gyro,
      DriveSubsystem driveSubsystem) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
    poseFunction = null;
    this.driveSubsystem = driveSubsystem;
  }

  public DriveToNearestWeight(
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d[]> targetPoses,
      Gyro gyro,
      Function<Pose2d, Pose2d> poseFunction,
      DriveSubsystem driveSubsystem) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
    this.poseFunction = poseFunction;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getPoseSpeeds(robotPose.get(), getNearestTarget(), gyro);
  }

  private Pose2d getNearestTarget() {
    if (poseFunction != null) {
      return DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get(), poseFunction);
    }
    return DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get());
  }

  public double getTargetDistance() {
    return robotPose.get().getTranslation().getDistance(getNearestTarget().getTranslation());
  }

  public Rotation2d getAngleDistance() {
    return getNearestTarget().getRotation().minus(robotPose.get().getRotation());
  }

  public boolean isAutoalignComplete() {
    Pose2d target = getNearestTarget();
    Pose2d robot = robotPose.get();
    boolean complete =
        (target.getTranslation().minus(robot.getTranslation()).getNorm() < 0.08
            && target.getRotation().minus(robot.getRotation()).getRadians() < Math.PI / 360);
    ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();
    complete &= chassisSpeeds.vxMetersPerSecond < 0.01 && chassisSpeeds.vyMetersPerSecond < 0.01;
    return complete;
  }
}
