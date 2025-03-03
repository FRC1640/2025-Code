package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.helpers.AutoAlignHelper;
import frc.robot.util.misc.DistanceManager;
import java.util.function.Supplier;

public class DriveToPointWeight implements DriveWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d> targetPose;
  private Gyro gyro;
  AutoAlignHelper autoAlignHelper = new AutoAlignHelper();

  public DriveToPointWeight(Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose, Gyro gyro) {
    this.robotPose = robotPose;
    this.targetPose = targetPose;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getPoseSpeedsLine(robotPose.get(), targetPose.get(), gyro);
  }

  public double getTargetDistance() {
    return DistanceManager.getPositionDistance(robotPose.get(), targetPose.get());
  }
}
