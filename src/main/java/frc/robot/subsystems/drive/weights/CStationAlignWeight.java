package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveCommandFactory;
import frc.robot.util.helpers.AutoAlignHelper;
import frc.robot.util.misc.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class CStationAlignWeight implements DriveWeight {
  private Supplier<Pose2d[]> positions;
  private Function<Pose2d, Pose2d> poseFunction;
  private Supplier<Pose2d> robotPose;
  private Pose2d targetPose;
  private AutoAlignHelper autoAlignHelper;
  private DriveSubsystem driveSubsystem;
  private DriveCommandFactory driveCommandFactory;
  private Gyro gyro;

  public CStationAlignWeight(
      Supplier<Pose2d[]> positions,
      Function<Pose2d, Pose2d> poseFunction,
      Supplier<Pose2d> robotPose,
      DriveSubsystem driveSubsystem,
      DriveCommandFactory driveCommandFactory,
      Gyro gyro) {
    this.positions = positions;
    this.robotPose = robotPose;
    this.targetPose = new Pose2d();
    this.autoAlignHelper = new AutoAlignHelper();
    this.driveSubsystem = driveSubsystem;
    this.driveCommandFactory = driveCommandFactory;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getCStationAlignSpeedsLine(
        robotPose.get(), targetPose, gyro, driveSubsystem);
  }

  private Pose2d findNearest(Pose2d[] pos) {
    if (poseFunction != null) {
      return DistanceManager.getNearestPosition(robotPose.get(), pos, poseFunction);
    }
    return DistanceManager.getNearestPosition(robotPose.get(), pos);
  }

  @Override
  public void onStart() {
    // create targetPose
    Pose2d nearest = findNearest(positions.get());
    Pose2d end =
        new Pose2d(nearest.getTranslation(), nearest.getRotation().plus(new Rotation2d(Math.PI)));
    // override rotation
    targetPose = findNearest(positions.get());
    autoAlignHelper.resetCStationMotionProfile(robotPose.get(), driveSubsystem);
  }

  public boolean isAutoalignComplete() {
    boolean complete =
        (targetPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.15
            && Math.abs(targetPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 6);
    ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();
    complete &= Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) < 0.01;
    return complete;
  }

  public Command getAutoCommand() {
    return driveCommandFactory
        .runVelocityCommand(() -> getSpeeds(), () -> true)
        .finallyDo(
            () -> driveCommandFactory.runVelocityCommand(() -> new ChassisSpeeds(), () -> true));
  }
}
