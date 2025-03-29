package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.util.helpers.AprilTagAlignHelper;
import frc.robot.util.helpers.AutoAlignHelper;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Pose2d> targetPose;
  private Supplier<Rotation2d> robotRotation;
  private AutoAlignHelper autoAlignHelper;
  private AprilTagVision[] visions;

  // TODO Trapezoidal Constraints????????
  public LocalTagAlignWeight(
      Supplier<Pose2d> targetPose, Supplier<Rotation2d> robotRotation, AprilTagVision... visions) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.visions = visions;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Optional<Translation2d> localAlignVector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    Logger.recordOutput("A_DEBUG/hello", localAlignVector.isPresent());
    return autoAlignHelper.getLocalAlignSpeedsLine(
        localAlignVector.isPresent() ? localAlignVector.get() : new Translation2d(),
        robotRotation.get(),
        AprilTagAlignHelper.getAutoalignTagId(targetPose.get())
            .pose
            .toPose2d()
            .getRotation()
            .minus(Rotation2d.kPi));
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }
}
