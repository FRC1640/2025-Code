package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.weights.DriveWeight;
import java.util.function.Supplier;

public class DriveToNearestWeight implements DriveWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d[]> targetPoses;
  private Gyro gyro;
  AutoAlignHelper autoAlignHelper = new AutoAlignHelper();

  public DriveToNearestWeight(
      Supplier<Pose2d> robotPose, Supplier<Pose2d[]> targetPoses, Gyro gyro) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getPoseSpeeds(
        robotPose.get(), autoAlignHelper.findNearest(targetPoses.get(), robotPose.get()), gyro);
  }
}
