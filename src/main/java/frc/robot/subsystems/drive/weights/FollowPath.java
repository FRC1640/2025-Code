package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.tools.DistanceManager;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowPath {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d[]> targetPoses;
  private Gyro gyro;
  private Function<Pose2d, Pose2d> poseFunction;

  Command pathCommand = null;
  private DriveSubsystem driveSubsystem;
  private Pose2d[] pose2dArray;

  public FollowPath(
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d[]> targetPoses,
      Gyro gyro,
      Function<Pose2d, Pose2d> poseFunction,
      DriveSubsystem driveSubsystem,
      Pose2d[] pose2dArray) {
    this.robotPose = robotPose;
    this.targetPoses = targetPoses;
    this.gyro = gyro;
    this.poseFunction = poseFunction;
    this.driveSubsystem = driveSubsystem;
    this.pose2dArray = pose2dArray;
  }

  public void stopPath() {
    pathCommand.cancel();
    PathplannerWeight.setSpeeds(new ChassisSpeeds());
    pathCommand = null;
  }

  public void startPath() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            List.of(robotPose.get(), pose2dArray).toArray(Pose2d[]::new));

    PathConstraints pathConstraints =
        new PathConstraints(DriveConstants.maxSpeed, 3, DriveConstants.maxOmega, 4 * Math.PI);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    path.preventFlipping = true;
    if (pathCommand == null) {
      pathCommand = AutoBuilder.followPath(path);
      pathCommand.schedule();
    }
  }

  public Trigger generateTrigger(BooleanSupplier condition) {
    return new Trigger(condition)
        .onTrue(new InstantCommand(() -> startPath()))
        .onFalse(new InstantCommand(() -> stopPath()));
  }

  private Pose2d getNearestTarget() {
    if (poseFunction != null) {
      return DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get(), poseFunction);
    }
    return DistanceManager.getNearestPosition(robotPose.get(), targetPoses.get());
  }

  public double getTargetDistance() {
    return robotPose.get().getTranslation().getDistance(getNearestTarget().getTranslation());
  }

  public Rotation2d getAngleDistance() {
    return getNearestTarget().getRotation().minus(robotPose.get().getRotation());
  }
}
