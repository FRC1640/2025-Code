package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.helpers.AutoAlignHelper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Pose2d> targetPose;
  private Supplier<Rotation2d> robotRotation;
  private AutoAlignHelper autoAlignHelper;

  // TODO Trapezoidal Constraints????????
  public LocalTagAlignWeight(Supplier<Pose2d> targetPose, Supplier<Rotation2d> robotRotation) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Logger.recordOutput(
        "A_DEBUG/end rotation",
        AutoAlignHelper.getAutoalignTagId(targetPose.get()).pose.toPose2d().getRotation());
    return autoAlignHelper.getLocalAlignSpeedsLine(
        RobotOdometry.getAverageLocalAlignVector().isPresent()
            ? RobotOdometry.getAverageLocalAlignVector().get()
            : new Translation2d(),
        robotRotation.get(),
        AutoAlignHelper.getAutoalignTagId(targetPose.get())
            .pose
            .toPose2d()
            .getRotation()
            .minus(Rotation2d.kPi));
  }

  public int getTargetTagId() {
    return AutoAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }
}
