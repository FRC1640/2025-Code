package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.tools.AutoAlignHelper;
import frc.robot.util.tools.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class DriveToNearestWeight extends AutoalignWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d[]> targetPoses;
  private Gyro gyro;
  private Function<Pose2d, Pose2d> poseFunction;
  AutoAlignHelper autoAlignHelper = new AutoAlignHelper();

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
    if (poseFunction != null) {
      return autoAlignHelper.getPoseSpeeds(
          robotPose.get(),
          DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get(), poseFunction),
          gyro);
    }
    return autoAlignHelper.getPoseSpeeds(
        robotPose.get(),
        DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get()),
        gyro);
  }

  public double getTargetDistance() {
    return DistanceManager.getNearestPositionDistance(robotPose.get(), targetPoses.get());
  }
}
