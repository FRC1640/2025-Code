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
    Optional<Translation2d> vector = AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isEmpty()) {
      return new ChassisSpeeds();
    }
    return autoAlignHelper.getLocalAlignSpeedsLine(
        vector.get(),
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
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }

  @Override
  public void onStart() {
    Optional<Translation2d> vector = AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      autoAlignHelper.resetLocalMotionProfile(vector.get(), driveSubsystem);
    } else {
      autoAlignHelper.resetLocalMotionProfile(new Translation2d(), driveSubsystem);
    }
  }

  public boolean isReady() {
    if (localAlignVector.isEmpty()
        && lastVector.isPresent()
        && currentIterations <= maxIterations) {
      currentIterations++;
    } else {
      lastVector = localAlignVector;
      currentIterations = 0;
    }
    localAlignVector = AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    // log vector presence
    // Logger.recordOutput("LocalTagAlign/isVectorPresent", localAlignVector.isPresent());
    // Logger.recordOutput("LocalVector", localAlignVector.get());
    // Logger.recordOutput("LastVector", lastVector.get());
    Optional<Translation2d> vector = Optional.empty();
    if (localAlignVector.isPresent()) {
      vector = localAlignVector;
    } else if (lastVector.isPresent()) {
      vector = lastVector;
    } else {
      Logger.recordOutput("vecpresent", vector.isPresent());
      return false;
    }
    Logger.recordOutput("vecpresent", vector.isPresent());
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
  }

  public Command getAutoCommand() {
    return driveCommandFactory
        .runVelocityCommand(() -> getSpeeds(), () -> true)
        .finallyDo(
            () -> driveCommandFactory.runVelocityCommand(() -> new ChassisSpeeds(), () -> true));
  }

  public boolean isAutoalignComplete() {
    Optional<Translation2d> vector = Optional.empty();
    if (localAlignVector.isPresent()) {
      vector = localAlignVector;
    } else if (lastVector.isPresent()) {
      vector = lastVector;
    } else {
      return false;
    }
    if (vector.isPresent()) {
      return vectorDeadband(vector.get());
    } else {
      return false;
    }
  }

  private boolean vectorDeadband(Translation2d vector) {
    return Math.abs(vector.getX()) < 0.03 && Math.abs(vector.getY()) < 0.03;
  }
}
