package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isEmpty()) {
      return new ChassisSpeeds();
    }
    return autoAlignHelper.getLocalAlignSpeedsLine(
        vector.get(),
        gyro,
        new Rotation2d((robotRotation.get().getRadians())),
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
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      autoAlignHelper.resetLocalMotionProfile(vector.get(), driveSubsystem);
    } else {
      autoAlignHelper.resetLocalMotionProfile(new Translation2d(), driveSubsystem);
    }
  }

  public boolean isReady() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    boolean ready = false;
    if (vector.isPresent()) {
      double goalAngle = // a break point triggers here as soon as the robot is connected -- as in,
          // when disabled, not just auto!
          FieldConstants.aprilTagLayout
              .getTagPose(getTargetTagId())
              .get()
              .toPose2d()
              .getRotation()
              .plus(Rotation2d.k180deg)
              .getRadians();

      Logger.recordOutput(
          "angledelta",
          Math.abs((robotRotation.get().minus(new Rotation2d(goalAngle))).getDegrees()));
      ready =
          vector.get().getNorm()
                  < 1 // and then ready is still false because the vector to the reef is still
              // greater than one
              && Math.abs((robotRotation.get().minus(new Rotation2d(goalAngle))).getDegrees()) < 15;
    }
    Logger.recordOutput("LocalTagAlign/isAlignReady", ready);
    Logger.recordOutput(
        "LocalTagAlign/vectorNorm", vector.isPresent() ? vector.get().getNorm() : -1);
    return ready;
  }

  public Command getAutoCommand() {
    return new InstantCommand(() -> System.out.println("Before runVelocityCommand"))
        .andThen(driveCommandFactory.runVelocityCommand(() -> getSpeeds(), () -> true))
        .andThen(() -> System.out.println("After runVelocityCommand"))
        .finallyDo(
            () -> driveCommandFactory.runVelocityCommand(() -> new ChassisSpeeds(), () -> true));
  }

  public void beans() {
    System.out.println("BeforeBEnassn");
    System.out.println("beans");
  }

  public boolean isAutoalignComplete() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      Rotation2d rotationError =
          new Rotation2d((robotRotation.get().getRadians()))
              .minus(
                  new Rotation2d(
                      FieldConstants.aprilTagLayout
                          .getTagPose(getTargetTagId())
                          .get()
                          .getRotation()
                          .toRotation2d()
                          .minus(Rotation2d.kPi)
                          .getRadians()));
      return vectorDeadband(vector.get()) && Math.abs(rotationError.getDegrees()) < 3;
    } else {
      return false;
    }
  }

  private boolean vectorDeadband(Translation2d vector) {
    return Math.abs(vector.getX()) < 0.025 && Math.abs(vector.getY()) < 0.025;
  }
}
