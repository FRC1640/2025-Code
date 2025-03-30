package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.subsystems.drive.DriveSubsystem;
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
  private DriveSubsystem driveSubsystem;

  private Translation2d lastVector = new Translation2d();

  public LocalTagAlignWeight(
      Supplier<Pose2d> targetPose,
      Supplier<Rotation2d> robotRotation,
      DriveSubsystem driveSubsystem,
      AprilTagVision... visions) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.visions = visions;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Optional<Translation2d> localAlignVector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    Logger.recordOutput("A_DEBUG/hello", localAlignVector.isPresent());
    if (localAlignVector.isPresent()) {
      lastVector = localAlignVector.get();
    }
    if (!lastVector.equals(new Translation2d())) {
      return autoAlignHelper.getLocalAlignSpeedsLine(
          lastVector,
          robotRotation.get(),
          AprilTagAlignHelper.getAutoalignTagId(targetPose.get())
              .pose
              .toPose2d()
              .getRotation()
              .minus(Rotation2d.kPi));
    } else return new ChassisSpeeds();
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }

  @Override
  public void onStart() {
    lastVector = new Translation2d();
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      lastVector = vector.get();
    }
    if (!lastVector.equals(new Translation2d())) {
      autoAlignHelper.resetLocalMotionProfile(lastVector, driveSubsystem);
    }
  }

  public boolean isReady() {
    Optional<Translation2d> vector = AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      lastVector = vector.get();
    }
    boolean ready = lastVector.getNorm() < 1.5 && vector.isPresent();
    Logger.recordOutput("A_DEBUG/vector present", vector.isPresent());
    Logger.recordOutput("A_DEBUG/localAlignReady", ready);
    return ready;
  }
}
