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
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.gyro.Gyro;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FollowPath {
  protected Supplier<Pose2d> robotPose;
  protected Gyro gyro;

  protected Command pathCommand = null;
  public Pose2d[] pose2dArray;
  public Rotation2d endRotation;
  private PathConstraints pathConstraints;

  public FollowPath(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] pose2dArray,
      PathConstraints pathConstraints,
      Rotation2d endRotation) {
    this.robotPose = robotPose;
    this.gyro = gyro;
    this.pose2dArray = pose2dArray;
    this.pathConstraints = pathConstraints;
    this.endRotation = endRotation;
    new Trigger(() -> !isNearSetpoint() && pathCommand != null)
        .onTrue(new InstantCommand(() -> restartPath()));
  }

  public void restartPath() {
    stopPath();

    startPath();
  }

  public void stopPath() {
    if (pathCommand != null) {
      pathCommand.cancel();
    }
    PathplannerWeight.setSpeeds(new ChassisSpeeds());
    pathCommand = null;
  }

  public void startPath() {
    List<Waypoint> waypoints;
    ArrayList<Pose2d> waypointPos = new ArrayList<>();
    for (Pose2d waypointPose2d : pose2dArray) {
      Logger.recordOutput("waypointPose2d", waypointPose2d.getRotation().getDegrees());
      waypointPos.add(waypointPose2d);
    }
    if (waypointPos.size() <= 0) {
      return;
    }
    if (robotPose.get().getTranslation().getDistance(waypointPos.get(0).getTranslation()) < 0.01) {
      return;
    }
    Rotation2d angle =
        waypointPos.get(0).getTranslation().minus(robotPose.get().getTranslation()).getAngle();
    waypointPos.add(0, new Pose2d(robotPose.get().getTranslation(), angle));

    waypoints = PathPlannerPath.waypointsFromPoses(waypointPos);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.00, endRotation));
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

  public boolean isNearSetpoint() {
    return PathplannerWeight.setpoint.getTranslation().getDistance(robotPose.get().getTranslation())
        < AutoAlignConfig.maxDistanceFromTarget;
  }
}
