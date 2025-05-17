package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveCommandFactory;
import frc.robot.util.helpers.AprilTagAlignHelper;
import frc.robot.util.helpers.AutoAlignHelper;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LocalPassiveAlignWeight implements DriveWeight {
  private Supplier<Pose2d> targetPose;
  private Supplier<Rotation2d> robotRotation;
  private AutoAlignHelper autoAlignHelper;
  private AprilTagVision[] visions;
  private DriveSubsystem driveSubsystem;
  private DriveCommandFactory driveCommandFactory;
  private Gyro gyro;
  private BooleanSupplier hasCoral;

  private static final double ASSIST_THRESH_BASE = 0.3;

  public LocalPassiveAlignWeight(
      Supplier<Pose2d> targetPose,
      Supplier<Rotation2d> robotRotation,
      DriveSubsystem driveSubsystem,
      DriveCommandFactory driveCommandFactory,
      Gyro gyro,
      BooleanSupplier hasCoral,
      AprilTagVision... visions) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.visions = visions;
    this.driveSubsystem = driveSubsystem;
    this.driveCommandFactory = driveCommandFactory;
    this.gyro = gyro;
    this.hasCoral = hasCoral;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    // average vectors across cameras
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isEmpty()) {
      return new ChassisSpeeds();
    }
    return autoAlignHelper.getPassiveLocalSpeeds(
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
                .getRadians()));
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }

  public boolean isReady() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    boolean ready = false;
    if (vector.isPresent()) {
      double goalAngle =
          FieldConstants.aprilTagLayout
              .getTagPose(getTargetTagId())
              .get()
              .toPose2d()
              .getRotation()
              .plus(Rotation2d.k180deg)
              .getRadians();
      ready =
          vector.get().getNorm() < 1
              && Math.abs((robotRotation.get().minus(new Rotation2d(goalAngle))).getDegrees()) < 15;
    }
    Logger.recordOutput("LocalTagAlign/isPassiveAlignReady", ready);
    return ready;
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

  @Override
  public double getWeight() {
    double weight;
    Optional<Translation2d> vector = AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      if (vector.get().getNorm() > AutoAlignConfig.reefAssistDistThresh || hasCoral.getAsBoolean()) {
        weight = 0;
      } else {
        double dist = vector.get().getNorm();
        weight = ASSIST_THRESH_BASE * (1 - 1 / (1 + Math.pow(Math.E, dist)));
      }
    } else {
      weight = 0;
    }
    Logger.recordOutput("LocalTagAlign/persistentWeight", weight);
    return weight;
  }
}
