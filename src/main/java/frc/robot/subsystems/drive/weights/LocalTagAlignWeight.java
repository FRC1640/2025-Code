package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveCommandFactory;
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
  private DriveCommandFactory driveCommandFactory;
  private Gyro gyro;

  private Translation2d lastVector = new Translation2d();

  public LocalTagAlignWeight(
      Supplier<Pose2d> targetPose,
      Supplier<Rotation2d> robotRotation,
      DriveSubsystem driveSubsystem,
      DriveCommandFactory driveCommandFactory,
      Gyro gyro,
      AprilTagVision... visions) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.visions = visions;
    this.driveSubsystem = driveSubsystem;
    this.driveCommandFactory = driveCommandFactory;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    // average vectors across cameras
    Optional<Translation2d> localAlignVector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    // log vector presence
    Logger.recordOutput("LocalTagAlign/isVectorPresent", localAlignVector.isPresent());
    // drive if vector present
    if (localAlignVector.isPresent()) {
      lastVector = localAlignVector.get();
    }
    if (!lastVector.equals(new Translation2d())) {
      return autoAlignHelper.getLocalAlignSpeedsLine(
          lastVector,
          gyro,
          new Rotation2d(MathUtil.angleModulus(robotRotation.get().getRadians())),
          new Rotation2d(
              FieldConstants.aprilTagLayout
                  .getTagPose(getTargetTagId())
                  .get()
                  .getRotation()
                  .toRotation2d()
                  .minus(Rotation2d.kPi)
                  .getRadians()),
          driveSubsystem);
    } else return new ChassisSpeeds();
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }

  @Override
  public void onStart() {
    autoAlignHelper.resetLocalMotionProfile(lastVector, driveSubsystem);
  }

  public boolean isReady() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      lastVector = vector.get();
      double goalAngle =
          MathUtil.angleModulus(
              FieldConstants.aprilTagLayout
                  .getTagPose(getTargetTagId())
                  .get()
                  .toPose2d()
                  .getRotation()
                  .unaryMinus()
                  .getRadians());
      boolean ready =
          vector.get().getNorm() < 1
              && MathUtil.angleModulus(robotRotation.get().getRadians()) - goalAngle < Math.PI / 18;
      Logger.recordOutput("LocalTagAlign/isAlignReady", ready);
      return ready;
    } else return false;
  }

  public Command getAutoCommand() {
    return driveCommandFactory
        .runVelocityCommand(() -> getSpeeds(), () -> true)
        .finallyDo(
            () -> driveCommandFactory.runVelocityCommand(() -> new ChassisSpeeds(), () -> true));
  }

  public boolean isAutoalignComplete() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      return vector.get().equals(new Translation2d());
    } else {
      return lastVector.equals(new Translation2d());
    }
  }
}
