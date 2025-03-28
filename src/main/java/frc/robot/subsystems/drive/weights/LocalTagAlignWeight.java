package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.helpers.AutoAlignHelper;
import java.util.function.Supplier;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Pose2d> targetPose;
  private Gyro gyro;
  private AutoAlignHelper autoAlignHelper;

  // TODO Trapezoidal Constraints????????
  public LocalTagAlignWeight(Supplier<Pose2d> targetPose, Gyro gyro) {
    this.gyro = gyro;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getLocalAlignSpeedsLine(
        RobotOdometry.getAverageLocalAlignVector(),
        gyro,
        AutoAlignHelper.getAutoalignTagId(targetPose.get()).pose.toPose2d().getRotation());
  }
}
