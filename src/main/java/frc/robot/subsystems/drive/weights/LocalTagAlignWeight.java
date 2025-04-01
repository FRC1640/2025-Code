package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.Gyro;
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
  private Gyro gyro;

  private Translation2d lastVector = new Translation2d();
  private boolean running = false;

  public LocalTagAlignWeight(
      Supplier<Pose2d> targetPose,
      Supplier<Rotation2d> robotRotation,
      DriveSubsystem driveSubsystem,
      Gyro gyro,
      AprilTagVision... visions) {
    this.robotRotation = robotRotation;
    this.targetPose = targetPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.visions = visions;
    this.driveSubsystem = driveSubsystem;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Optional<Translation2d> localAlignVector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    Logger.recordOutput("A_DEBUG/hello", localAlignVector.isPresent());
    this.running = false;
    if (localAlignVector.isPresent()) {
      lastVector = localAlignVector.get();
    }
    if (!lastVector.equals(new Translation2d())) {
      this.running = true;
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
                  .getRadians()));
    } else return new ChassisSpeeds();
  }

  public int getTargetTagId() {
    return AprilTagAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }

  @Override
  public void onStart() {
    // System.out.println("hey there");
    // lastVector = new Translation2d();
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      lastVector = vector.get();
    }
    Logger.recordOutput("A_DEBUG/starting vector present?", vector.isPresent());
    if (!lastVector.equals(new Translation2d())) {
      autoAlignHelper.resetLocalMotionProfile(lastVector, driveSubsystem);
    }
  }

  public boolean isReady() {
    Optional<Translation2d> vector =
        AprilTagAlignHelper.getAverageLocalAlignVector(getTargetTagId(), visions);
    if (vector.isPresent()) {
      lastVector = vector.get();
    }
    Logger.recordOutput("A_DEBUG/ready vector present?", vector.isPresent());
    // System.out.println(lastVector);
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
        lastVector.getNorm() < 1
            && MathUtil.angleModulus(robotRotation.get().getRadians()) - goalAngle < Math.PI / 18;
    Logger.recordOutput("A_DEBUG/vector present", vector.isPresent());
    Logger.recordOutput("A_DEBUG/localAlignReady", ready);
    return ready;
  }

  @Override
  public boolean isEnabled() {
    return running;
  }
}
