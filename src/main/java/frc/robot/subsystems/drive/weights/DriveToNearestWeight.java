package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
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

  public DriveToNearestWeight(
      Supplier<Pose2d> robotPose, Supplier<Pose2d[]> targetPoses, Gyro gyro) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
    poseFunction = null;
  }

  public DriveToNearestWeight(
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d[]> targetPoses,
      Gyro gyro,
      Function<Pose2d, Pose2d> poseFunction) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
    this.poseFunction = poseFunction;
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

  @Override
  public boolean getEnabled() {
    return enabled;
  }

  @Override
  public void setEnabled(boolean setEnabled) {
    enabled = setEnabled;
  }
}
